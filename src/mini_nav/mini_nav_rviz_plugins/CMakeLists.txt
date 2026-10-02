cmake_minimum_required(VERSION 3.8)
project(mini_nav_rviz_plugins)

set(CMAKE_AUTOMOC ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
    src/navigation_status_panel.cpp
    include/mini_nav_rviz_plugins/navigation_status_panel.hpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} geometry_msgs nav2_msgs action_msgs pluginlib rclcpp rviz_common std_msgs)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_navigation_status_panel test/test_navigation_status_panel.cpp
        SKIP_LINKING_MAIN_LIBRARIES TIMEOUT 90)
    target_compile_features(test_navigation_status_panel PRIVATE cxx_std_17)
    ament_target_dependencies(test_navigation_status_panel geometry_msgs nav2_msgs action_msgs rclcpp rviz_common std_msgs)
    target_link_libraries(test_navigation_status_panel Qt5::Widgets)
    set_tests_properties(test_navigation_status_panel PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=212;QT_QPA_PLATFORM=xcb")
endif()

ament_package()
