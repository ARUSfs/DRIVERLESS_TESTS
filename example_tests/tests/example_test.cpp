#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"

class ExampleTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp::Node>("test_node");
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr node;
};

TEST_F(ExampleTest, TestNodeExists)
{
  ASSERT_NE(node, nullptr);
}

TEST_F(ExampleTest, TestPublishing)
{
  auto publisher = node->create_publisher<std_msgs::msg::String>("test_topic", 10);
  ASSERT_NE(publisher, nullptr);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
