cmake_minimum_required(VERSION 3.8)
project(example_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Optional dependencies (uncomment if needed)
# find_package(rosbag2_cpp REQUIRED)
# find_package(rclcpp_lifecycle REQUIRED)

include_directories(include)

add_executable(number_publisher_node
  src/number_publisher.cpp
)

ament_target_dependencies(number_publisher_node
  rclcpp
  std_msgs
)

install(TARGETS
  number_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data)



# Enable testing
# if(BUILD_TESTING)
#   enable_testing()

#   ament_add_gtest(example_test tests/example_test.cpp)
#   target_link_libraries(example_test
#         rclcpp::rclcpp)

#    ament_target_dependencies(example_test
#         rclcpp
#         std_msgs
#       )

#   ament_add_gtest(example_rosbag_test tests/example_rosbag_test.cpp)
#   target_link_libraries(example_rosbag_test
#         rclcpp::rclcpp)

#    ament_target_dependencies(example_rosbag_test
#         rclcpp
#         std_msgs
#       )
# endif()

ament_package()
