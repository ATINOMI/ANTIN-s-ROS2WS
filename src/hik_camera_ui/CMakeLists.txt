cmake_minimum_required(VERSION 3.16)
project(hik_camera_ui)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core Widgets)

add_executable(hik_camera_ui
  src/main.cpp
  src/ros_worker.cpp
  src/main_window.cpp
  include/hik_camera_ui/main_window.hpp
  include/hik_camera_ui/ros_worker.hpp
)

target_include_directories(hik_camera_ui PRIVATE
  include
)

target_link_libraries(hik_camera_ui
  Qt6::Core
  Qt6::Widgets
)

ament_target_dependencies(hik_camera_ui
  rclcpp
  sensor_msgs
  image_transport
)

install(TARGETS hik_camera_ui
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
