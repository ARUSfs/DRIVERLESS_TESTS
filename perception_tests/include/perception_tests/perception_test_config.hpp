// include/perception_tests/perception_test_config.hpp
#pragma once
#include <string>
#include <rclcpp/rclcpp.hpp>

namespace perception_tests
{

struct PerceptionConfig
{
    double kDistanceThreshold = 1; // m
    double kMinExpectedAccuracy = 0.9;
    double kMinDetectedAccuracy = 0.9;
    double kCroppingRadius = 40.0; // m

    std::string kPerceptionPackage = "perception";
    std::string kPerceptionLaunch = "perception_launch.py";

    std::string kLidarPointsTopic = "/rslidar_points";
    std::string kPerceptionMapTopic = "/perception/map";
    std::string kFinalMapTopic = "/slam/final_map";

    std::string kLidarFrame = "rslidar";
    std::string kWorldFrame = "arussim/world";
    std::string kVehicleFrame = "slam/vehicle";
};

inline PerceptionConfig load_config_from_node(const rclcpp::Node::SharedPtr &node)
{
    PerceptionConfig cfg;

    // Declare parameters with defaults (so they can be overridden if desired)
    node->declare_parameter("distance_threshold", cfg.kDistanceThreshold);
    node->declare_parameter("min_expected_accuracy", cfg.kMinExpectedAccuracy);
    node->declare_parameter("min_detected_accuracy", cfg.kMinDetectedAccuracy);
    node->declare_parameter("cropping_radius", cfg.kCroppingRadius);

    node->declare_parameter("perception_package", cfg.kPerceptionPackage);
    node->declare_parameter("perception_launch", cfg.kPerceptionLaunch);

    node->declare_parameter("lidar_points_topic", cfg.kLidarPointsTopic);
    node->declare_parameter("perception_map_topic", cfg.kPerceptionMapTopic);
    node->declare_parameter("final_map_topic", cfg.kFinalMapTopic);

    node->declare_parameter("lidar_frame", cfg.kLidarFrame);
    node->declare_parameter("world_frame", cfg.kWorldFrame);
    node->declare_parameter("vehicle_frame", cfg.kVehicleFrame);

    node->get_parameter("distance_threshold", cfg.kDistanceThreshold);
    node->get_parameter("min_expected_accuracy", cfg.kMinExpectedAccuracy);
    node->get_parameter("min_detected_accuracy", cfg.kMinDetectedAccuracy);
    node->get_parameter("cropping_radius", cfg.kCroppingRadius);

    node->get_parameter("perception_package", cfg.kPerceptionPackage);
    node->get_parameter("perception_launch", cfg.kPerceptionLaunch);

    node->get_parameter("lidar_points_topic", cfg.kLidarPointsTopic);
    node->get_parameter("perception_map_topic", cfg.kPerceptionMapTopic);
    node->get_parameter("final_map_topic", cfg.kFinalMapTopic);

    node->get_parameter("lidar_frame", cfg.kLidarFrame);
    node->get_parameter("world_frame", cfg.kWorldFrame);
    node->get_parameter("vehicle_frame", cfg.kVehicleFrame);

    return cfg;
}

} // namespace perception_tests