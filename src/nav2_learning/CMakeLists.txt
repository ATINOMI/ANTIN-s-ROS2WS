cmake_minimum_required(VERSION 3.8)
project(nav2_learning)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(hello_nav2 src/hello_nav2.cpp)
ament_target_dependencies(hello_nav2 rclcpp)

install(TARGETS
  hello_nav2
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
install(DIRECTORY launch urdf config models rviz DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch urdf DESTINATION share/${PROJECT_NAME})

