#include "example_tests/number_publisher.hpp"

NumberPublisher::NumberPublisher() : Node("number_publisher"), count_(1)
{
  publisher_ = this->create_publisher<std_msgs::msg::Int32>("/numbers", 10);
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&NumberPublisher::timer_callback, this));
}

void NumberPublisher::timer_callback()
{
  if (count_ > 10)
  {
    rclcpp::shutdown();
    return;
  }

  std_msgs::msg::Int32 msg;
  msg.data = count_++;
  RCLCPP_INFO(this->get_logger(), "Publishing: %d", msg.data);
  publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NumberPublisher>());
  rclcpp::shutdown();
  return 0;
}
