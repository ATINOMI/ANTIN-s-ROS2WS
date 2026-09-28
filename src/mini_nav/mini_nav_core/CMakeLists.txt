cmake_minimum_required(VERSION 3.8)
project(mini_nav_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME}
  src/map/costmap_2d.cpp
  src/map/inflation_layer.cpp
  src/navigator/astar_navigator.cpp
  src/localization/localization_map.cpp
  src/localization/differential_motion_model.cpp
  src/localization/likelihood_field_model.cpp
  src/localization/beam_model.cpp
  src/localization/kd_tree.cpp
  src/localization/particle_filter.cpp
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

  ament_add_gtest(test_costmap_2d test/test_costmap_2d.cpp)
  if(TARGET test_costmap_2d)
    target_link_libraries(test_costmap_2d ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_astar_planner test/test_astar_planner.cpp)
  if(TARGET test_astar_planner)
    target_link_libraries(test_astar_planner ${PROJECT_NAME})
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
