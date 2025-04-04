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

TEST_F(ExampleTest, TestPublisherSubscriber)
{
    std::string received_message;

    auto publisher = node->create_publisher<std_msgs::msg::String>("test_topic", 10);
    auto subscription = node->create_subscription<std_msgs::msg::String>(
        "test_topic", 10,
        [&received_message](const std_msgs::msg::String::SharedPtr msg)
        {
            received_message = msg->data;
        });

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std_msgs::msg::String msg;
    msg.data = "Hello, test!";
    publisher->publish(msg);

    // Spin for a short time to allow message to be received
    auto start = std::chrono::steady_clock::now();
    while (received_message.empty() &&
           std::chrono::steady_clock::now() - start < std::chrono::seconds(1))
    {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    ASSERT_EQ(received_message, "Hello, test!");
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
