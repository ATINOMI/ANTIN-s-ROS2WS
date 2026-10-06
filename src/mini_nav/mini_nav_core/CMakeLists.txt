cmake_minimum_required(VERSION 3.8)
project(mini_nav_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME}
  src/map/costmap_2d.cpp
  src/map/inflation_layer.cpp
  src/map/rolling_obstacle_grid.cpp
  src/navigator/astar_navigator.cpp
  src/navigator/path_postprocessor.cpp
  src/navigator/path_tracker.cpp
  src/localization/amcl/localization_map.cpp
  src/localization/amcl/differential_motion_model.cpp
  src/localization/amcl/likelihood_field_model.cpp
  src/localization/amcl/beam_model.cpp
  src/localization/amcl/kd_tree.cpp
  src/localization/amcl/particle_filter.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_collision_geometry test/test_collision_geometry.cpp)
  target_link_libraries(test_collision_geometry ${PROJECT_NAME})
  target_compile_definitions(test_collision_geometry PRIVATE
    COLLISION_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")

  # 官方库只作为膨胀回归测试的对照，不进入核心库的运行依赖。
  find_package(nav2_costmap_2d REQUIRED)
  find_package(nav2_util REQUIRED)
  find_package(tf2_ros REQUIRED)
  ament_add_gtest(test_inflation_layer test/test_inflation_layer.cpp)
  if(TARGET test_inflation_layer)
    target_link_libraries(test_inflation_layer ${PROJECT_NAME})
    ament_target_dependencies(test_inflation_layer nav2_costmap_2d nav2_util tf2_ros)
    find_library(NAV2_LAYERS_LIBRARY layers REQUIRED)
    target_link_libraries(test_inflation_layer ${NAV2_LAYERS_LIBRARY})
    target_compile_definitions(test_inflation_layer PRIVATE
      MINI_NAV_TEST_MAP="${CMAKE_CURRENT_SOURCE_DIR}/../mini_nav_bringup/maps/tb3_learning.pgm")
    set_tests_properties(test_inflation_layer PROPERTIES
      ENVIRONMENT "ROS_DOMAIN_ID=219;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
  endif()

  ament_add_gtest(test_costmap_2d test/test_costmap_2d.cpp)
  if(TARGET test_costmap_2d)
    target_link_libraries(test_costmap_2d ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_astar_planner test/test_astar_planner.cpp)
  if(TARGET test_astar_planner)
    target_link_libraries(test_astar_planner ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_rolling_obstacle_grid test/test_rolling_obstacle_grid.cpp)
  if(TARGET test_rolling_obstacle_grid)
    target_link_libraries(test_rolling_obstacle_grid ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_path_tracker test/test_path_tracker.cpp)
  if(TARGET test_path_tracker)
    target_link_libraries(test_path_tracker ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_particle_filter test/test_particle_filter.cpp)
  if(TARGET test_particle_filter)
    target_link_libraries(test_particle_filter ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_localization_naming test/test_localization_naming.cpp)
  if(TARGET test_localization_naming)
    target_link_libraries(test_localization_naming ${PROJECT_NAME})
  endif()

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
