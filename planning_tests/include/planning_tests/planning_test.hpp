#ifndef PLANNING_TEST_HPP_
#define PLANNING_TEST_HPP_

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <common_msgs/msg/trajectory.hpp>
#include <memory>
#include <string>
#include <sys/types.h>

class PathPlanningTest : public ::testing::TestWithParam<std::string>
{
protected:
    rclcpp::Node::SharedPtr node;
    std::shared_ptr<common_msgs::msg::Trajectory> final_trayectory;
    rclcpp::Subscription<common_msgs::msg::Trajectory>::SharedPtr path_planning_subscription;

    bool trajectory_received{false};
    double total_detected_accuracy{0.0};
    int evaluation_count{0};
    int timeout_count{0};
    pid_t path_planning_launch_pid{-1};

    void SetUp() override;
    void TearDown() override;

    void launch_path_planning();
    void kill_path_planning();
    void path_planning_callback(const common_msgs::msg::Trajectory::SharedPtr msg);
    void play_rosbag(const std::string &bag_path);
    std::shared_ptr<common_msgs::msg::Trajectory> get_last_trayectory(const std::string &bag_path);
};

#endif // PLANNING_TEST_HPP_