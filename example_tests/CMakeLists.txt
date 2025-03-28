cmake_minimum_required(VERSION 3.8)
project(example_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Optional dependencies (uncomment if needed)
# find_package(ament_cmake_gmock REQUIRED)
# find_package(rosbag2_cpp REQUIRED)
# find_package(rclcpp_lifecycle REQUIRED)

# Enable testing
if(BUILD_TESTING)
  enable_testing()  # ✅ Move this inside BUILD_TESTING

  ament_add_gtest(example_test tests/example_test.cpp)
  target_link_libraries(example_test
        rclcpp::rclcpp)

   ament_target_dependencies(example_test
        rclcpp
        std_msgs
      )

  # Uncomment for GMock test
  # ament_add_gmock(driverless_mock_test tests/test_mock_driverless.cpp)
  # target_link_libraries(driverless_mock_test rclcpp::rclcpp rosbag2_cpp::rosbag2_cpp)
endif()

ament_package()
