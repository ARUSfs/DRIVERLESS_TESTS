#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <chrono>
#include <thread>
#include <vector>
#include <ament_index_cpp/get_package_share_directory.hpp>

class RosbagPlaybackTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp::Node>("rosbag_playback_test_node");
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr node;
  std::vector<int32_t> received_numbers;
};

void play_numbers_rosbag()
{
  std::string package_share_directory = ament_index_cpp::get_package_share_directory("example_tests");
  std::string rosbag_path = package_share_directory + "/data/number_bag";
  std::string command = "ros2 bag play " + rosbag_path;
  system(command.c_str());

  // Sleep to allow time for the rosbag playback to finish
  std::this_thread::sleep_for(std::chrono::seconds(1));
}

TEST_F(RosbagPlaybackTest, TestNumbersSequence)
{
  received_numbers.clear();

  // Create a subscription to the /numbers topic
  auto subscription = node->create_subscription<std_msgs::msg::Int32>(
      "/numbers", 10,
      [this](const std_msgs::msg::Int32::SharedPtr msg)
      {
        RCLCPP_INFO(node->get_logger(), "Received: %d", msg->data);
        received_numbers.push_back(msg->data);
      });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spin_thread([&executor, this]()
                          {
    auto start = std::chrono::steady_clock::now();
    while (
      received_numbers.size() < 10 &&
      std::chrono::steady_clock::now() - start < std::chrono::seconds(5))
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } });

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  play_numbers_rosbag();

  spin_thread.join();

  ASSERT_EQ(received_numbers.size(), 10);

  for (int i = 0; i < 10; ++i)
  {
    ASSERT_EQ(received_numbers[i], i + 1);
  }
}
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
