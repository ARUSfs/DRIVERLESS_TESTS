#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <rmw/serialized_message.h>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <common_msgs/msg/trajectory.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <cstdlib>
#include <thread>
#include <chrono>
#include <sys/wait.h>
#include <iomanip>

class PathPlanningTest : public ::testing::TestWithParam<std::string>
{
protected:
    rclcpp::Node::SharedPtr node;
    std::shared_ptr<common_msgs::msg::Trajectory> final_trayectory;
    rclcpp::Subscription<common_msgs::msg::Trajectory>::SharedPtr path_planning_subscription;

    bool trajectory_received = false;
    double total_detected_accuracy = 0.0;
    int evaluation_count = 0;
    int timeout_count = 0;

    double kMinDetectedAccuracy = 0.9;
    double kDistanceThreshold = 0.4;
    std::string kPathPlanningPackage = "path_planning";
    std::string kPathPlanningLaunch = "path_planning_launch.py";
    std::string kTrayectoryTopic = "/path_planning/trajectory";
    std::string kPerceptionMapTopic = "/perception/map";

    pid_t path_planning_launch_pid = -1;

    void SetUp() override
    {
        rclcpp::init(0, nullptr);
        launch_path_planning();
        node = std::make_shared<rclcpp::Node>("rosbag_playback_test");
        final_trayectory = std::make_shared<common_msgs::msg::Trajectory>();

        path_planning_subscription = node->create_subscription<common_msgs::msg::Trajectory>(
            kTrayectoryTopic, 10,
            std::bind(&PathPlanningTest::path_planning_callback, this, std::placeholders::_1));
    }

    void TearDown() override
    {
        kill_path_planning();
        rclcpp::shutdown();
    }

    void launch_path_planning()
    {
        path_planning_launch_pid = fork();
        if (path_planning_launch_pid == 0)
        {
            execlp("ros2", "ros2", "launch", kPathPlanningPackage.c_str(), kPathPlanningLaunch.c_str(), (char *)nullptr);
            std::cerr << "Failed to launch path planning node\n";
            std::_Exit(EXIT_FAILURE);
        }
        else if (path_planning_launch_pid < 0)
        {
            FAIL() << "Failed to fork for launching path planning node";
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }
    }

    void kill_path_planning()
    {
        if (path_planning_launch_pid > 0)
        {
            kill(path_planning_launch_pid, SIGINT);
            waitpid(path_planning_launch_pid, nullptr, 0);
            path_planning_launch_pid = -1;
        }
    }

    void path_planning_callback(const common_msgs::msg::Trajectory::SharedPtr msg)
    {
        RCLCPP_INFO(node->get_logger(), "Trajectory callback triggered with %zu points", msg->points.size());

        if (msg->points.empty())
        {
            RCLCPP_WARN(node->get_logger(), "Received empty trajectory.");
            return;
        }

        if (!final_trayectory || final_trayectory->points.empty())
        {
            RCLCPP_WARN(node->get_logger(), "Final trajectory not loaded yet, skipping comparison.");
            return;
        }

        pcl::PointCloud<pcl::PointXY>::Ptr received_cloud(new pcl::PointCloud<pcl::PointXY>);
        pcl::PointCloud<pcl::PointXY>::Ptr expected_cloud(new pcl::PointCloud<pcl::PointXY>);

        for (const auto &pt : msg->points)
        {
            pcl::PointXY p;
            p.x = pt.x;
            p.y = pt.y;
            received_cloud->points.push_back(p);
        }

        for (const auto &pt : final_trayectory->points)
        {
            pcl::PointXY p;
            p.x = pt.x;
            p.y = pt.y;
            expected_cloud->points.push_back(p);
        }

        pcl::KdTreeFLANN<pcl::PointXY> kdtree;
        kdtree.setInputCloud(expected_cloud);

        std::vector<bool> expected_matched(expected_cloud->points.size(), false);
        int true_positive = 0;

        for (const auto &det : received_cloud->points)
        {
            std::vector<int> idx(1);
            std::vector<float> dist(1);

            if (kdtree.nearestKSearch(det, 1, idx, dist))
            {
                float distance = std::sqrt(dist[0]);
                if (distance < kDistanceThreshold && !expected_matched[idx[0]])
                {
                    expected_matched[idx[0]] = true;
                    true_positive++;
                }
            }
        }

        trajectory_received = true;

        double detected_accuracy = received_cloud->points.size() > 0
            ? static_cast<double>(true_positive) / received_cloud->points.size()
            : 0.0;

        total_detected_accuracy += detected_accuracy;
        evaluation_count++;
    }

    void play_rosbag(const std::string &bag_path)
    {
        rosbag2_cpp::readers::SequentialReader reader;
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = bag_path;
        storage_options.storage_id = "mcap";

        rosbag2_cpp::ConverterOptions converter_options{"cdr", "cdr"};
        reader.open(storage_options, converter_options);

        auto topics_and_types = reader.get_all_topics_and_types();
        std::unordered_map<std::string, std::string> topic_type_map;
        for (const auto &info : topics_and_types)
        {
            topic_type_map[info.name] = info.type;
        }

        std::unordered_map<std::string, rclcpp::GenericPublisher::SharedPtr> publishers;
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);

        while (reader.has_next())
        {
            auto bag_msg = reader.read_next();
            const std::string &topic = bag_msg->topic_name;

            if (topic == kTrayectoryTopic)
            {
                continue;
            }

            if (publishers.find(topic) == publishers.end())
            {
                auto it = topic_type_map.find(topic);
                if (it == topic_type_map.end())
                {
                    RCLCPP_WARN(node->get_logger(), "Unknown type for topic: %s", topic.c_str());
                    continue;
                }

                auto publisher = node->create_generic_publisher(topic, it->second, 10);
                publishers[topic] = publisher;
            }

             rclcpp::SerializedMessage msg(*bag_msg->serialized_data);
            publishers[topic]->publish(msg);

            if (topic == kPerceptionMapTopic.c_str())
            {
                auto start = std::chrono::steady_clock::now();
                int kTimeOut = 10;
                while (!trajectory_received &&
                       std::chrono::steady_clock::now() - start < std::chrono::seconds(kTimeOut))
                {
                    executor.spin_some();
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                RCLCPP_INFO(node->get_logger(), "Waited for answer");
                if (!trajectory_received)
                {
                  timeout_count++;
                  RCLCPP_INFO(node->get_logger(), "Timed out!");
                  if(timeout_count >= 5){

                    timeout_count = 0;
                    RCLCPP_INFO(node->get_logger(), "Timed out many times!");
                    break;

                  }
                }

                // Reset
                trajectory_received = false;
            }


        }
    }

   std::shared_ptr<common_msgs::msg::Trajectory> get_last_trayectory(const std::string &bag_path)
{
    RCLCPP_INFO(node->get_logger(), "Opening bag file at path: %s", bag_path.c_str());

    rosbag2_cpp::readers::SequentialReader reader;
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path;
    storage_options.storage_id = "mcap";

    rosbag2_cpp::ConverterOptions converter_options{"cdr", "cdr"};

    try
    {
        reader.open(storage_options, converter_options);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to open rosbag: %s", e.what());
        return nullptr;
    }

    rclcpp::Serialization<common_msgs::msg::Trajectory> serializer;
    std::shared_ptr<common_msgs::msg::Trajectory> last_trajectory_msg = nullptr;

    size_t message_count = 0;

    while (reader.has_next())
    {
        auto bag_msg = reader.read_next();
        message_count++;


        if (bag_msg->topic_name == kTrayectoryTopic)
        {

            auto msg = std::make_shared<common_msgs::msg::Trajectory>();
            rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
            serializer.deserialize_message(&serialized_msg, msg.get());
            last_trajectory_msg = msg;

        }
    }

    if (!last_trajectory_msg)
    {
        RCLCPP_WARN(node->get_logger(), "No trajectory message found in the rosbag.");
    }
    else
    {
        RCLCPP_INFO(node->get_logger(), "Last trajectory message successfully loaded.");
    }

    return last_trajectory_msg;
}

};

TEST_P(PathPlanningTest, PerceptionAccuracyTest)
{
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("planning_tests");
    std::string rosbag_path = package_share_directory + "/data/" + GetParam();
    final_trayectory = get_last_trayectory(rosbag_path);

    try
    {
        play_rosbag(rosbag_path);
    }
    catch (const std::exception &e)
    {
        FAIL() << "Exception during rosbag playback: " << e.what();
    }
    catch (...)
    {
        FAIL() << "Unknown exception during rosbag playback.";
    }

    ASSERT_GT(evaluation_count, 0) << "No trajectory messages received or callback not triggered.";

    double mean_detected_accuracy = total_detected_accuracy / evaluation_count;

    RCLCPP_INFO(node->get_logger(), "======FINAL DATA======");
    RCLCPP_INFO(node->get_logger(), "Mean detected accuracy: %.2f%%", mean_detected_accuracy * 100);

    std::ostringstream stream_mean_detected_accuracy;
    stream_mean_detected_accuracy << std::fixed << std::setprecision(5) << mean_detected_accuracy;
    ::testing::Test::RecordProperty("MeanDetectedAccuracy", stream_mean_detected_accuracy.str());

    ASSERT_GE(mean_detected_accuracy, kMinDetectedAccuracy) << "Mean detection accuracy too low!";
}

INSTANTIATE_TEST_SUITE_P(
    PathPlanning,
    PathPlanningTest,
    ::testing::Values(
        "path_planning_test.mcap"
    )
);

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
