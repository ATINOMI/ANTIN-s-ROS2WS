cmake_minimum_required(VERSION 3.8)
project(mini_nav_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(mini_nav_core REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(costmap_publisher_node
  src/main.cpp
  src/costmap_publisher.cpp
)

target_compile_definitions(costmap_publisher_node PRIVATE MINI_NAV_BUILD_COSTMAP=1)

add_executable(mini_nav_amcl_node
  src/main.cpp
  src/amcl_node.cpp
)

target_compile_definitions(mini_nav_amcl_node PRIVATE MINI_NAV_BUILD_AMCL=1)

target_compile_features(costmap_publisher_node PUBLIC cxx_std_17)
target_include_directories(costmap_publisher_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(costmap_publisher_node
  nav_msgs
  geometry_msgs
  rclcpp
  visualization_msgs
)
target_link_libraries(costmap_publisher_node mini_nav_core::mini_nav_core)
target_compile_features(mini_nav_amcl_node PUBLIC cxx_std_17)
target_include_directories(mini_nav_amcl_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(mini_nav_amcl_node
  geometry_msgs
  message_filters
  mini_nav_core
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_link_libraries(mini_nav_amcl_node mini_nav_core::mini_nav_core)

install(TARGETS costmap_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS mini_nav_amcl_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
