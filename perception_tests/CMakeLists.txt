cmake_minimum_required(VERSION 3.8)
project(perception_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)



# Optional dependencies (uncomment if needed)

# include_directories(include)

# install(DIRECTORY include/
#   DESTINATION include/
# )

install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data)



# Enable testing
if(BUILD_TESTING)
  enable_testing()

  ament_add_gtest(perception_test tests/perception_test.cpp)
  target_link_libraries(perception_test
        rclcpp::rclcpp)

   ament_target_dependencies(perception_test
        rclcpp
        rosbag2_cpp
        rosbag2_storage
        sensor_msgs
        std_msgs
        tf2
        tf2_ros
        tf2_sensor_msgs
        PCL
        pcl_conversions
      )

endif()

ament_package()
