cmake_minimum_required(VERSION 3.8)
project(mini_nav_pf_debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mini_nav_core REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(pf_debug_node src/main.cpp)
target_compile_features(pf_debug_node PUBLIC cxx_std_17)
ament_target_dependencies(pf_debug_node
  ament_index_cpp geometry_msgs mini_nav_core nav_msgs rclcpp sensor_msgs tf2
  tf2_geometry_msgs tf2_ros
)
target_link_libraries(pf_debug_node mini_nav_core::mini_nav_core)

install(TARGETS pf_debug_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch web DESTINATION share/${PROJECT_NAME})

ament_package()
