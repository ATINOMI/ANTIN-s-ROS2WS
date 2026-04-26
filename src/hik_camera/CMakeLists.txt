cmake_minimum_required(VERSION 3.8)
project(hik_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(hik_camera_node src/hik_camera_node.cpp)

ament_target_dependencies(hik_camera_node
  rclcpp
  sensor_msgs
  camera_info_manager
  image_transport
  OpenCV
)

target_include_directories(hik_camera_node PRIVATE /opt/MVS/include)
target_link_libraries(hik_camera_node /opt/MVS/lib/64/libMvCameraControl.so ${OpenCV_LIBS})

install(TARGETS hik_camera_node
DESTINATION lib/${PROJECT_NAME}
)


ament_package()
