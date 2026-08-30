cmake_minimum_required(VERSION 3.8)
project(mini_nav_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# This package contains Python launch files and configuration only.  ROS 2's
# launch packages are runtime dependencies, not CMake packages.
install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

# RViz configurations are project-wide assets kept in the parent mini_nav/rviz directory.
install(DIRECTORY ../rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
  PATTERN "*.orig" EXCLUDE
)

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

ament_package()
