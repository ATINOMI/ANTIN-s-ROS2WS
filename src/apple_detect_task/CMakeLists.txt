cmake_minimum_required(VERSION 3.8)
project(apple_detect_task)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(image_publisher_node
  src/image_publisher.cpp
  src/main.cpp
)

add_executable(monitor_node
  src/monitor_node.cpp
  src/monitor_main.cpp
  src/detector.cpp
)

target_include_directories(image_publisher_node PRIVATE include)

ament_target_dependencies(image_publisher_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)

target_include_directories(monitor_node PRIVATE include)

ament_target_dependencies(monitor_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)

target_link_libraries(image_publisher_node ${OpenCV_LIBRARIES})
target_link_libraries(monitor_node ${OpenCV_LIBRARIES})

install(TARGETS
  image_publisher_node
  monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
