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
find_package(visualization_msgs REQUIRED)

add_executable(costmap_publisher_node
  src/main.cpp
  src/costmap_publisher.cpp
)

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

install(TARGETS costmap_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
