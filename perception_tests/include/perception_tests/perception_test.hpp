// include/perception_tests/perception_test.hpp
#pragma once

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "perception_test_config.hpp"
#include "perception/PointXYZIRingTime.h"

namespace perception_tests
{

class PerceptionTest : public ::testing::TestWithParam<std::string>
{
protected:
    // lifecycle
    void SetUp() override;
    void TearDown() override;

    // helpers
    void launch_perception();
    void kill_perception();
    void create_static_tf();
    void play_rosbag(const std::string &bag_path);
    void perception_callback(sensor_msgs::msg::PointCloud2::SharedPtr detected_cones);
    void cropping(sensor_msgs::msg::PointCloud2 &transformed_cloud);

    // members
    rclcpp::Node::SharedPtr node;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_subscription;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_map_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr expected_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr detected_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr matched_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr unmatched_pub;

    sensor_msgs::msg::PointCloud2 final_map;
    bool map_received = false;
    rclcpp::Time time_of_last_lidar_message;

    // metrics
    double total_detected_accuracy = 0.0;
    double total_expected_accuracy = 0.0;
    int evaluation_count = 0;

    // pid
    pid_t perception_launch_pid = -1;

    // config
    PerceptionConfig cfg;
};

} // namespace perception_tests