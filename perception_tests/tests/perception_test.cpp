// src/perception_test.cpp
#include "perception_tests/perception_test.hpp"

#include <rclcpp/serialization.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_cloud.h>

#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <sys/wait.h>
#include <signal.h>
#include <chrono>
#include <thread>
#include <sstream>
#include <iomanip>

struct TrackedCone {
    Eigen::Vector2f pos;
    int age;
};

std::vector<TrackedCone> tracked_cones;

using namespace perception_tests;

void PerceptionTest::SetUp()
{
    // init ROS context for tests
    rclcpp::init(0, nullptr);

    node = std::make_shared<rclcpp::Node>("rosbag_playback_test_node");

    // load config from node parameters (can be overridden)
    cfg = load_config_from_node(node);

    launch_perception();

    create_static_tf();

    map_received = false;

    map_subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        cfg.kPerceptionMapTopic, 10,
        std::bind(&PerceptionTest::perception_callback, this, std::placeholders::_1));

    tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    transformed_map_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/test/final_map", 10);
    expected_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/test/expected", 10);
    detected_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/test/detected", 10);
    matched_pub  = node->create_publisher<sensor_msgs::msg::PointCloud2>("/test/matched", 10);
    unmatched_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/test/unmatched", 10);
}

void PerceptionTest::TearDown()
{
    kill_perception();
    rclcpp::shutdown();
}

void PerceptionTest::create_static_tf()
{
    static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

    geometry_msgs::msg::TransformStamped static_transform;
    static_transform.header.stamp = node->now();
    static_transform.header.frame_id = cfg.kVehicleFrame;
    static_transform.child_frame_id = cfg.kLidarFrame;

    static_transform.transform.translation.x = 0.0;
    static_transform.transform.translation.y = 0.0;
    static_transform.transform.translation.z = 0.0;
    static_transform.transform.rotation.x = 0.0;
    static_transform.transform.rotation.y = 0.0;
    static_transform.transform.rotation.z = 0.0;
    static_transform.transform.rotation.w = 1.0;

    static_broadcaster->sendTransform(static_transform);
}

void PerceptionTest::launch_perception()
{
    perception_launch_pid = fork();
    if (perception_launch_pid == 0)
    {
        // child
        execlp("ros2", "ros2", "launch", cfg.kPerceptionPackage.c_str(), cfg.kPerceptionLaunch.c_str(), (char *)nullptr);
        std::cerr << "Failed to exec ros2 launch\n";
        _exit(EXIT_FAILURE);
    }
    else if (perception_launch_pid < 0)
    {
        FAIL() << "Could not fork to launch perception node";
    }
    else
    {
        // parent wait a bit
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }
}

void PerceptionTest::kill_perception()
{
    if (perception_launch_pid > 0)
    {
        kill(perception_launch_pid, SIGINT);
        waitpid(perception_launch_pid, nullptr, 0);
        perception_launch_pid = -1;
    }
}

void PerceptionTest::play_rosbag(const std::string &bag_path)
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
        topic_type_map[info.name] = info.type;

    std::unordered_map<std::string, rclcpp::GenericPublisher::SharedPtr> publishers;

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    while (reader.has_next())
    {
        auto bag_msg = reader.read_next();
        const std::string &topic = bag_msg->topic_name;

        if (final_map.data.empty())
        {
            if (topic == cfg.kFinalMapTopic)
            {
                rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
                sensor_msgs::msg::PointCloud2 msg;
                rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
                serializer.deserialize_message(&serialized_msg, &msg);
                time_of_last_lidar_message = msg.header.stamp;
                final_map = msg;
                // continue to next messages (we need final_map to transform expected)
            }
            else
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
            publishers[topic] = node->create_generic_publisher(topic, it->second, 10);
        }

        rclcpp::SerializedMessage msg(*bag_msg->serialized_data);
        publishers[topic]->publish(msg);

        if (topic == cfg.kLidarPointsTopic)
        {
            // Deserialize message to get timestamp
            rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);

            sensor_msgs::msg::PointCloud2 msg;
            rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
            serializer.deserialize_message(&serialized_msg, &msg);

            time_of_last_lidar_message = msg.header.stamp;
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
            map_received = false;
        }
    }
}

void PerceptionTest::cropping(sensor_msgs::msg::PointCloud2 &transformed_cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(transformed_cloud, *pcl_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr half_cylinder_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    double angle_min = -M_PI / 3;
    double angle_max = M_PI / 3;

    for (const auto &point : pcl_cloud->points)
    {
        float distance_xy = std::sqrt(point.x * point.x + point.y * point.y);
        if (distance_xy <= cfg.kCroppingRadius)
        {
            float angle = std::atan2(point.y, point.x);
            if (angle >= angle_min && angle <= angle_max)
                half_cylinder_filtered->points.push_back(point);
        }
    }

    half_cylinder_filtered->width = half_cylinder_filtered->points.size();
    half_cylinder_filtered->height = 1;
    half_cylinder_filtered->is_dense = true;

    pcl::toROSMsg(*half_cylinder_filtered, transformed_cloud);
    transformed_cloud.header.frame_id = cfg.kLidarFrame;
    transformed_cloud.header.stamp = node->now();
}

void filter_by_fov(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const PerceptionConfig& cfg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);

    double angle_min = -M_PI / 3;  // 120° FOV
    double angle_max =  M_PI / 3;

    for (const auto &point : cloud->points)
    {
        float distance_xy = std::sqrt(point.x * point.x + point.y * point.y);

        if (distance_xy <= cfg.kCroppingRadius)
        {
            float angle = std::atan2(point.y, point.x);

            if (angle >= angle_min && angle <= angle_max)
            {
                filtered->points.push_back(point);
            }
        }
    }

    filtered->width = filtered->points.size();
    filtered->height = 1;
    filtered->is_dense = true;

    cloud = filtered;
}

void PerceptionTest::perception_callback(sensor_msgs::msg::PointCloud2::SharedPtr detected_cones)
{
    ASSERT_FALSE(detected_cones->data.empty()) << "Cone data was empty";
    map_received = true;

    if (final_map.data.empty())
    {
        RCLCPP_WARN(node->get_logger(), "Final map not loaded yet, skipping transform.");
        return;
    }

    try
    {
        geometry_msgs::msg::TransformStamped transform_stamped =
            tf_buffer->lookupTransform(
                cfg.kLidarFrame,
                cfg.kWorldFrame,
                tf2::TimePointZero);

        sensor_msgs::msg::PointCloud2 transformed_cloud;
        tf2::doTransform(final_map, transformed_cloud, transform_stamped);

        cropping(transformed_cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr matched_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr unmatched_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // Convert to PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr expected_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr detected_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(transformed_cloud, *expected_cloud);
        pcl::fromROSMsg(*detected_cones, *detected_cloud);

        filter_by_fov(expected_cloud, cfg);
        filter_by_fov(detected_cloud, cfg);

        for (auto &pt : expected_cloud->points) pt.z = 0;
        for (auto &pt : detected_cloud->points) pt.z = 0;

        // AFTER converting clouds and setting z=0

        // 🔁 Reinitialize tracking EVERY FRAME (for testing)

        for (const auto &p : expected_cloud->points) {
            TrackedCone c;
            c.pos = Eigen::Vector2f(p.x, p.y);
            c.age = 0;
            tracked_cones.push_back(c);
        }

        // Build KDTree from tracked cones
        pcl::PointCloud<pcl::PointXYZ>::Ptr tracked_cloud(new pcl::PointCloud<pcl::PointXYZ>());

        for (const auto &c : tracked_cones) {
            pcl::PointXYZ pt;
            pt.x = c.pos.x();
            pt.y = c.pos.y();
            pt.z = 0;
            tracked_cloud->points.push_back(pt);
        }

        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(tracked_cloud);

        std::vector<bool> matched(tracked_cones.size(), false);

        int true_positive = 0;
        int false_positive = 0;

        const float GATING_DIST = cfg.kDistanceThreshold; // reuse your config

        // MATCH DETECTED -> TRACKED
        for (const auto &det : detected_cloud->points)
        {
            pcl::PointXYZ search_pt;
            search_pt.x = det.x;
            search_pt.y = det.y;
            search_pt.z = 0;

            std::vector<int> idx(1);
            std::vector<float> dist(1);

            if (kdtree.nearestKSearch(search_pt, 1, idx, dist))
            {
                if (dist[0] < GATING_DIST * GATING_DIST && !matched[idx[0]])
                {
                    pcl::PointXYZ pt;
                    pt.x = det.x;
                    pt.y = det.y;
                    pt.z = 0;
                    matched_cloud->points.push_back(pt);

                    matched[idx[0]] = true;
                    true_positive++;

                    // UPDATE TRACKED POSITION (low-pass filter)
                    tracked_cones[idx[0]].pos = Eigen::Vector2f(det.x, det.y);

                    tracked_cones[idx[0]].age = 0;
                }
                else
                {
                    pcl::PointXYZ pt;
                    pt.x = det.x;
                    pt.y = det.y;
                    pt.z = 0;
                    unmatched_cloud->points.push_back(pt);
                    false_positive++;
                }
            }
            else
            {
                pcl::PointXYZ pt;
                pt.x = det.x;
                pt.y = det.y;
                pt.z = 0;
                unmatched_cloud->points.push_back(pt);
                false_positive++;
            }
        }

        // AGE TRACKS
        for (auto &c : tracked_cones) {
            c.age++;
        }
        
        // REMOVE OLD TRACKS (robust for tests)
        tracked_cones.erase(
            std::remove_if(tracked_cones.begin(), tracked_cones.end(),
                        [](const TrackedCone &c) { return c.age > 15; }),
            tracked_cones.end());

        int total_detected = detected_cloud->points.size();
        int total_expected = expected_cloud->points.size();

        double detected_accuracy = total_detected > 0 ? static_cast<double>(true_positive) / total_detected : 0.0;
        double expected_accuracy = total_expected > 0 ? static_cast<double>(true_positive) / total_expected : 0.0;

        RCLCPP_INFO(node->get_logger(), "Detected accuracy: %.2f%%", detected_accuracy * 100);
        RCLCPP_INFO(node->get_logger(), "Expected accuracy: %.2f%%", expected_accuracy * 100);

        // ACCUMULATE
        this->total_detected_accuracy += detected_accuracy;
        this->total_expected_accuracy += expected_accuracy;
        this->evaluation_count++;
        
        auto publish_cloud = [&](pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub)
                        {
                            sensor_msgs::msg::PointCloud2 msg;
                            pcl::toROSMsg(*cloud, msg);
                            msg.header.frame_id = cfg.kLidarFrame;
                            msg.header.stamp = node->now();
                            pub->publish(msg);
                        };
        // Convert expected_cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr expected_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        for (auto &p : expected_cloud->points)
            expected_xyz->points.emplace_back(p.x, p.y, 0);
        
        RCLCPP_INFO(node->get_logger(),
            "TP: %d | FP: %d | Expected: %d | Detected: %d",
            true_positive, false_positive,
            total_expected, total_detected);
        
        // Convert detected_cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr detected_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        for (auto &p : detected_cloud->points)
            detected_xyz->points.emplace_back(p.x, p.y, 0);

        RCLCPP_INFO(node->get_logger(),
            "TP: %d | FP: %d | Expected: %d | Detected: %d",
            true_positive, false_positive,
            total_expected, total_detected);

        // Publish
        publish_cloud(expected_xyz, expected_pub);
        publish_cloud(detected_xyz, detected_pub);
        publish_cloud(matched_cloud, matched_pub);
        publish_cloud(unmatched_cloud, unmatched_pub);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(node->get_logger(), "Could not transform final map: %s", ex.what());
        return;
    }
}

// TEST and main
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
        FAIL() << "Exception during rosbag playback: " << e.what() << " Rosbag test: " << rosbag_path;
    }

    double mean_detected_accuracy = total_detected_accuracy / std::max(1, evaluation_count);
    double mean_expected_accuracy = total_expected_accuracy / std::max(1, evaluation_count);

    RCLCPP_INFO(node->get_logger(), "======FINAL DATA======");
    RCLCPP_INFO(node->get_logger(), "Mean detected accuracy: %.2f%%", mean_detected_accuracy * 100);
    RCLCPP_INFO(node->get_logger(), "Mean expected accuracy: %.2f%%", mean_expected_accuracy * 100);

    std::ostringstream stream_mean_detected_accuracy;
    stream_mean_detected_accuracy << std::fixed << std::setprecision(5) << mean_detected_accuracy;

    std::ostringstream stream_mean_expected_accuracy;
    stream_mean_expected_accuracy << std::fixed << std::setprecision(5) << mean_expected_accuracy;

    ::testing::Test::RecordProperty("MeanDetectedAccuracy", stream_mean_detected_accuracy.str());
    ::testing::Test::RecordProperty("MeanExpectedAccuracy", stream_mean_expected_accuracy.str());

    ASSERT_GE(mean_detected_accuracy, cfg.kMinDetectedAccuracy) << "Mean detection accuracy too low!";
    ASSERT_GE(mean_expected_accuracy, cfg.kMinExpectedAccuracy) << "Mean expected accuracy too low!";
}

INSTANTIATE_TEST_SUITE_P(Perception, PerceptionTest, ::testing::Values("trackdrive_bag_2026-04-10_15-37-47.mcap"));

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
