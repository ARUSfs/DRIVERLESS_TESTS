cmake_minimum_required(VERSION 3.16)
project(perception_tests)

# Warnings extra para GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------
# Find dependencies
# -------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(gtest_vendor REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)

# -------------------
# Include directories
# -------------------
# include de este paquete
include_directories(
  include
  # ruta al header de perception
  ../DRIVERLESS2/src/perception/include
  ${PCL_INCLUDE_DIRS}
)

install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data)


# -------------------
# Add gtest
# -------------------
ament_add_gtest(perception_test
  tests/perception_test.cpp
)

# Añadir includes y dependencias al target
target_include_directories(perception_test
  PRIVATE
    include
    ../../../DRIVERLESS2/src/perception/include
    ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(perception_test
  rclcpp
  sensor_msgs
  rosbag2_cpp
  rosbag2_storage
  ament_index_cpp
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
  PCL
)

target_link_libraries(perception_test ${PCL_LIBRARIES})

# -------------------
# Habilitar salida XML para gtest
# -------------------
if(NOT DEFINED ENV{GTEST_OUTPUT})
  set(ENV{GTEST_OUTPUT} "xml:$CMAKE_BINARY_DIR/test_results/")
endif()

# -------------------
# Instalar
# -------------------
install(TARGETS perception_test DESTINATION lib/${PROJECT_NAME})

ament_package()
