/**
  ******************************************************************************
  * @file      monitor_main.cpp
  * @author    ANTINOMI
  * @date      2026-07-30
  * @brief     monitor_node 的程序入口。
  *            本文件负责初始化 ROS 2、创建 ImageMonitor 节点并进入事件循环。
  ******************************************************************************
  * @attention
  *
  * 本文件属于 apple_detect_task ROS 2 功能包。
  ******************************************************************************
  */

#include <memory>

#include "apple_detect_task/monitor_node.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief 图像监测节点程序入口。
 *
 * @param argc 命令行参数数量。
 * @param argv 命令行参数数组。
 * @return 程序退出状态。
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageMonitor>());
  rclcpp::shutdown();
  return 0;
}
