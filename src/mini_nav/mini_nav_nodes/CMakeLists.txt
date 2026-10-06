cmake_minimum_required(VERSION 3.8)
project(mini_nav_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(bondcpp REQUIRED)
find_package(mini_nav_core REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "msg/CollisionMap.msg"
  DEPENDENCIES nav_msgs geometry_msgs)
rosidl_get_typesupport_target(collision_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(costmap_publisher_node
  src/main.cpp
  src/map_manager/costmap_publisher.cpp
)

target_compile_definitions(costmap_publisher_node PRIVATE MINI_NAV_BUILD_COSTMAP=1)

add_executable(mini_nav_amcl_node
  src/main.cpp
  src/amcl/amcl_node.cpp
)

add_executable(local_costmap_node
  src/main.cpp
  src/map_manager/local_costmap_node.cpp
)
target_compile_definitions(local_costmap_node PRIVATE MINI_NAV_BUILD_LOCAL_COSTMAP=1)
target_compile_features(local_costmap_node PUBLIC cxx_std_17)
target_include_directories(local_costmap_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(local_costmap_node
  geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros
)
target_link_libraries(local_costmap_node mini_nav_core::mini_nav_core)
target_link_libraries(local_costmap_node "${collision_typesupport}")

add_executable(path_follower_node
  src/main.cpp
  src/tracker_manager/path_follower_node.cpp
)
target_compile_definitions(path_follower_node PRIVATE MINI_NAV_BUILD_PATH_FOLLOWER=1)
target_compile_features(path_follower_node PUBLIC cxx_std_17)
target_include_directories(path_follower_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(path_follower_node
  geometry_msgs nav_msgs nav2_msgs rclcpp rclcpp_action std_msgs tf2 tf2_geometry_msgs tf2_ros
)
target_link_libraries(path_follower_node mini_nav_core::mini_nav_core)
target_link_libraries(path_follower_node "${collision_typesupport}")

target_compile_definitions(mini_nav_amcl_node PRIVATE MINI_NAV_BUILD_AMCL=1)

target_compile_features(costmap_publisher_node PUBLIC cxx_std_17)
target_include_directories(costmap_publisher_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(costmap_publisher_node
  sensor_msgs
  nav_msgs
  nav2_msgs
  geometry_msgs
  rclcpp_action
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)
target_link_libraries(costmap_publisher_node mini_nav_core::mini_nav_core)
target_link_libraries(costmap_publisher_node "${collision_typesupport}")
target_compile_features(mini_nav_amcl_node PUBLIC cxx_std_17)
target_include_directories(mini_nav_amcl_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(mini_nav_amcl_node
  bondcpp
  geometry_msgs
  message_filters
  mini_nav_core
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)
target_link_libraries(mini_nav_amcl_node mini_nav_core::mini_nav_core)

install(TARGETS costmap_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS mini_nav_amcl_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS local_costmap_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS path_follower_node
  DESTINATION lib/${PROJECT_NAME}
)

foreach(component navigation_manager velocity_guard)
  string(TOUPPER ${component} component_upper)
  if(component STREQUAL "navigation_manager")
    set(component_directory navigation_task_manager)
  else()
    set(component_directory velocity_guard)
  endif()
  add_executable(${component}_node src/main.cpp src/${component_directory}/${component}_node.cpp)
  target_compile_definitions(${component}_node PRIVATE MINI_NAV_BUILD_${component_upper}=1)
  target_compile_features(${component}_node PUBLIC cxx_std_17)
  target_include_directories(${component}_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(${component}_node geometry_msgs nav2_msgs rclcpp rclcpp_action std_msgs std_srvs nav_msgs tf2 tf2_geometry_msgs tf2_ros)
  install(TARGETS ${component}_node DESTINATION lib/${PROJECT_NAME})
endforeach()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_test(NAME test_costmap_safety COMMAND /usr/bin/python3
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_costmap_safety.py ${CMAKE_CURRENT_BINARY_DIR})
  set_tests_properties(test_costmap_safety PROPERTIES TIMEOUT 35
    ENVIRONMENT "ROS_DOMAIN_ID=216;RMW_IMPLEMENTATION=rmw_cyclonedds_cpp;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
  add_test(NAME test_navigation_integration COMMAND /usr/bin/python3
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_navigation_integration.py ${CMAKE_CURRENT_BINARY_DIR})
  set_tests_properties(test_navigation_integration PROPERTIES TIMEOUT 160
    ENVIRONMENT "ROS_DOMAIN_ID=217;RMW_IMPLEMENTATION=rmw_cyclonedds_cpp")

  ament_add_gtest(test_planner_dynamic_obstacles
    test/test_planner_dynamic_obstacles.cpp src/map_manager/costmap_publisher.cpp)
  target_compile_features(test_planner_dynamic_obstacles PUBLIC cxx_std_17)
  target_include_directories(test_planner_dynamic_obstacles PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_planner_dynamic_obstacles geometry_msgs nav_msgs nav2_msgs sensor_msgs
    rclcpp rclcpp_action tf2 tf2_geometry_msgs tf2_ros visualization_msgs)
  target_link_libraries(test_planner_dynamic_obstacles mini_nav_core::mini_nav_core)
  target_link_libraries(test_planner_dynamic_obstacles "${collision_typesupport}")

  ament_add_gtest(test_amcl_tf_cache
    test/test_amcl_tf_cache.cpp
    src/amcl/amcl_node.cpp
  )
  if(TARGET test_amcl_tf_cache)
    target_compile_features(test_amcl_tf_cache PUBLIC cxx_std_17)
    target_include_directories(test_amcl_tf_cache PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    ament_target_dependencies(test_amcl_tf_cache
      bondcpp
      geometry_msgs
      message_filters
      mini_nav_core
      nav2_msgs
      nav_msgs
      rclcpp
      rclcpp_lifecycle
      sensor_msgs
      std_msgs
      std_srvs
      tf2
      tf2_geometry_msgs
      tf2_ros
      visualization_msgs
    )
    target_link_libraries(test_amcl_tf_cache mini_nav_core::mini_nav_core)
  endif()

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
