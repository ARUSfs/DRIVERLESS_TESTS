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

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

#include <memory>
#include <unordered_map>
#include <unordered_set>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <pcl/kdtree/kdtree_flann.h>

#include <cstdlib>
#include <thread>
#include <chrono>

class PerceptionTest : public ::testing::TestWithParam<std::string>
{
protected:
    // Variables
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster;

    rclcpp::Node::SharedPtr node;
    std::unordered_set<std::string> allowed_topics;
    sensor_msgs::msg::PointCloud2 final_map;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_subscription;

    bool map_received = false;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_map_pub;

    double total_detected_accuracy = 0.0;
    double total_expected_accuracy = 0.0;
    int evaluation_count = 0;

    rclcpp::Time time_of_last_lidar_message;

    // Configuration

    double kDistanceThreshold = 0.75; // meters
    double kMinExpectedAccuracy = 0.4;
    double kMinDetectedAccuracy = 0.9;
    double kCroppingRadius = 20.0; // meters

    std::string kPerceptionPackage = "perception";
    std::string kPerceptionLaunch = "perception_launch.py";

    std::string kLidarPointsTopic = "/rslidar_points";
    std::string kPerceptionMapTopic = "/perception/map";
    std::string kFinalMapTopic = "/slam/final_map";

    std::string kLidarFrame = "rslidar";
    std::string kWorldFrame = "arussim/world";
    std::string kVehicleFrame = "slam/vehicle";

    void
    SetUp() override
    {

        rclcpp::init(0, nullptr);

        launch_perception();
        // Create a test node
        node = std::make_shared<rclcpp::Node>("rosbag_playback_test");

        create_static_tf();

        // Initialize Variables
        map_received = false;

        // Set up a subscription to /perception/map
        map_subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
            kPerceptionMapTopic.c_str(),
            10,
            std::bind(&PerceptionTest::perception_callback, this, std::placeholders::_1));

        tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        transformed_map_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/test/final_map", 10);
    }

    void create_static_tf()
    {
        static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

        // Create static transform: slam/vehicle → rslidar
        geometry_msgs::msg::TransformStamped static_transform;
        static_transform.header.stamp = node->now();
        static_transform.header.frame_id = kVehicleFrame.c_str();
        static_transform.child_frame_id = kLidarFrame.c_str();

        // Set the relative position of rslidar on the vehicle (example values)
        static_transform.transform.translation.x = 0.0;
        static_transform.transform.translation.y = 0.0;
        static_transform.transform.translation.z = 0.0;

        static_transform.transform.rotation.x = 0.0;
        static_transform.transform.rotation.y = 0.0;
        static_transform.transform.rotation.z = 0.0;
        static_transform.transform.rotation.w = 1.0;

        static_broadcaster->sendTransform(static_transform);
    }

    void TearDown() override
    {
        kill_perception();
        rclcpp::shutdown();
    }

    pid_t perception_launch_pid = -1;

    void launch_perception()
    {
        perception_launch_pid = fork();
        if (perception_launch_pid == 0)
        {
            // Child process: run ros2 launch
            execlp("ros2", "ros2", "launch", kPerceptionPackage.c_str(), kPerceptionLaunch.c_str(), (char *)nullptr);
            std::cerr << "Failed to launch perception node\n";
            std::_Exit(EXIT_FAILURE);
        }
        else if (perception_launch_pid < 0)
        {
            FAIL() << "Failed to fork for launching perception node";
        }
        else
        {
            // Parent process: give it time to start
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }
    }

    void kill_perception()
    {
        if (perception_launch_pid > 0)
        {
            kill(perception_launch_pid, SIGINT);
            waitpid(perception_launch_pid, nullptr, 0);
            perception_launch_pid = -1;
        }
    }

    void perception_callback(sensor_msgs::msg::PointCloud2 detected_cones)
    {
        ASSERT_FALSE(detected_cones.data.empty()) << "Cone data was empty";
        map_received = true;

        if (final_map.data.empty())
        {
            RCLCPP_WARN(node->get_logger(), "Final map not loaded yet, skipping transform.");
            return;
        }

        try
        {
            geometry_msgs::msg::TransformStamped transform_stamped =
                tf_buffer->lookupTransform(kLidarFrame.c_str(), kWorldFrame.c_str(), time_of_last_lidar_message, tf2::durationFromSec(1.0));

            sensor_msgs::msg::PointCloud2 transformed_cloud;
            tf2::doTransform(final_map, transformed_cloud, transform_stamped);

            cropping(transformed_cloud);

            // Convert to PCL
            pcl::PointCloud<pcl::PointXYZ>::Ptr expected_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr detected_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(transformed_cloud, *expected_cloud);
            pcl::fromROSMsg(detected_cones, *detected_cloud);

            // Remove Z component: just zero it out to treat it as 2D
            for (auto &pt : expected_cloud->points)
                pt.z = 0;
            for (auto &pt : detected_cloud->points)
                pt.z = 0;

            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
            kdtree.setInputCloud(expected_cloud);

            std::vector<bool> expected_matched(expected_cloud->points.size(), false);
            int true_positive = 0;
            int false_positive = 0;

            for (const auto &det : detected_cloud->points)
            {
                std::vector<int> idx(1);
                std::vector<float> dist(1);
                if (kdtree.nearestKSearch(det, 1, idx, dist))
                {
                    if (dist[0] < kDistanceThreshold && !expected_matched[idx[0]])
                    {
                        expected_matched[idx[0]] = true;
                        true_positive++;
                    }
                    else
                    {
                        false_positive++;
                    }
                }
                else
                {
                    false_positive++;
                }
            }

            int total_detected = detected_cloud->points.size();
            int total_expected = expected_cloud->points.size();
            int false_negative = std::count(expected_matched.begin(), expected_matched.end(), false);

            double detected_accuracy = total_detected > 0 ? static_cast<double>(true_positive) / total_detected : 0.0;
            double expected_accuracy = total_expected > 0 ? static_cast<double>(true_positive) / total_expected : 0.0;

            RCLCPP_INFO(node->get_logger(), "Detected accuracy: %.2f%%", detected_accuracy * 100);
            RCLCPP_INFO(node->get_logger(), "Expected accuracy: %.2f%%", expected_accuracy * 100);

            // Publish result
            transformed_map_pub->publish(transformed_cloud);

            total_detected_accuracy += detected_accuracy;
            total_expected_accuracy += expected_accuracy;
            evaluation_count++;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(node->get_logger(), "Could not transform final map: %s", ex.what());
            return;
        }
    }

    void cropping(sensor_msgs::msg::PointCloud2 &transformed_cloud)
    {
        // Convert ROS msg to PCL cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(transformed_cloud, *pcl_cloud);

        // Cylindrical + angular filtering
        pcl::PointCloud<pcl::PointXYZI>::Ptr half_cylinder_filtered(new pcl::PointCloud<pcl::PointXYZI>);

        // Angle range in radians (e.g., -90° to +90°)
        double angle_min = -M_PI / 2; // -90 degrees
        double angle_max = M_PI / 2;  // +90 degrees

        for (const auto &point : pcl_cloud->points)
        {
            float distance_xy = std::sqrt(point.x * point.x + point.y * point.y);
            if (distance_xy <= kCroppingRadius)
            {
                float angle = std::atan2(point.y, point.x); // angle in XY plane
                if (angle >= angle_min && angle <= angle_max)
                {
                    half_cylinder_filtered->points.push_back(point);
                }
            }
        }

        half_cylinder_filtered->width = half_cylinder_filtered->points.size();
        half_cylinder_filtered->height = 1;
        half_cylinder_filtered->is_dense = true;

        // Convert back to ROS msg
        pcl::toROSMsg(*half_cylinder_filtered, transformed_cloud);
        transformed_cloud.header.frame_id = "rslidar";
        transformed_cloud.header.stamp = node->now();
    }

    void play_rosbag(const std::string &bag_path)
    {

        // Setup a rosbag reader
        rosbag2_cpp::readers::SequentialReader reader;
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = bag_path;
        storage_options.storage_id = "mcap";

        rosbag2_cpp::ConverterOptions converter_options{"cdr", "cdr"};
        reader.open(storage_options, converter_options);

        // Preregister Topics and Types
        auto topics_and_types = reader.get_all_topics_and_types();
        std::unordered_map<std::string, std::string> topic_type_map;
        for (const auto &info : topics_and_types)
        {
            topic_type_map[info.name] = info.type;
        }

        std::unordered_map<std::string, rclcpp::GenericPublisher::SharedPtr> publishers;

        // Set up the executor
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);

        while (reader.has_next())
        {
            auto bag_msg = reader.read_next();
            const std::string &topic = bag_msg->topic_name;

            // Skip until the final map has information
            if (final_map.data.empty())
            {

                if (topic == kFinalMapTopic.c_str())
                {

                    // Deserialize to actual PointCloud2 message
                    rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
                    sensor_msgs::msg::PointCloud2 msg;
                    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
                    serializer.deserialize_message(&serialized_msg, &msg);
                    // Save to final_map
                    final_map = msg;
                }
                else
                {
                    continue;
                }
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

            if (topic == kLidarPointsTopic.c_str())
            {
                time_of_last_lidar_message = rclcpp::Time(0);
                // Spin until /perception/map is received
                auto start = std::chrono::steady_clock::now();
                int kTimeOut = 20;
                while (!map_received &&
                       std::chrono::steady_clock::now() - start < std::chrono::seconds(kTimeOut))
                {
                    executor.spin_some();
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }

                if (!map_received)
                {
                    FAIL() << "Timeout waiting for /perception/map response.";
                }

                // Reset
                map_received = false;
            }
        }
    }
};

TEST_P(PerceptionTest, PerceptionAccuracyTest)
{
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("perception_tests");
    std::string rosbag_path = package_share_directory + "/data/" + GetParam();
    try
    {
        play_rosbag(rosbag_path);
    }
    catch (const std::exception &e)
    {
        FAIL() << "Exception caught during rosbag playback: " << e.what();
    }
    catch (...)
    {
        FAIL() << "Unknown exception caught during rosbag playback.";
    }

    double mean_detected_accuracy = total_detected_accuracy / evaluation_count;
    double mean_expected_accuracy = total_expected_accuracy / evaluation_count;

    RCLCPP_INFO(node->get_logger(), "======FINAL DATA======");
    RCLCPP_INFO(node->get_logger(), "Mean detected accuracy: %.2f%%", mean_detected_accuracy * 100);
    RCLCPP_INFO(node->get_logger(), "Mean expected accuracy: %.2f%%", mean_expected_accuracy * 100);

    std::ostringstream stream_mean_detected_accuracy;
    stream_mean_detected_accuracy << std::fixed << std::setprecision(5) << mean_detected_accuracy;

    std::ostringstream stream_mean_expected_accuracy;
    stream_mean_expected_accuracy << std::fixed << std::setprecision(5) << mean_expected_accuracy;

    ::testing::Test::RecordProperty("MeanDetectedAccuracy", stream_mean_detected_accuracy.str());
    ::testing::Test::RecordProperty("MeanExpectedAccuracy", stream_mean_expected_accuracy.str());


    ASSERT_GE(mean_detected_accuracy, kMinDetectedAccuracy) << "Mean detection accuracy too low!";
    ASSERT_GE(mean_expected_accuracy, kMinExpectedAccuracy) << "Mean expected accuracy too low!";
}

INSTANTIATE_TEST_SUITE_P(
    Perception,     // Instance name
    PerceptionTest, // Test suite name
    ::testing::Values(
        "cartuja.mcap"));

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
