cmake_minimum_required(VERSION 3.8)
project(nav2_stvl_demo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(scan_to_cloud src/scan_to_cloud.cpp)
ament_target_dependencies(scan_to_cloud rclcpp sensor_msgs)

install(TARGETS scan_to_cloud DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config urdf models DESTINATION share/${PROJECT_NAME})

ament_package()
