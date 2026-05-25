cmake_minimum_required(VERSION 3.8)
project(planning_tests)

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
find_package(common_msgs REQUIRED)

# Instalar el directorio data para que GetParam() encuentre el .mcap en share/
install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data)



# Enable testing
if(BUILD_TESTING)
  enable_testing()

  # Añadimos el ejecutable del test
  ament_add_gtest(planning_test tests/planning_test.cpp)
  
  # ¡CRUCIAL!: Decirle a CMake dónde buscar tus archivos .hpp
  target_include_directories(planning_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  )

  target_link_libraries(planning_test
    rclcpp::rclcpp
  )

  ament_target_dependencies(planning_test
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
    common_msgs
  )

endif()

ament_package()