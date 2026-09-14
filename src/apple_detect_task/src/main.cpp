/**
  ******************************************************************************
  * @file      main.cpp
  * @author    ANTINOMI
  * @date      2026-07-30
  * @brief     image_publisher_node 的程序入口。
  *            本文件负责初始化 ROS 2、创建 ImagePublisher 节点并进入事件循环。
  ******************************************************************************
  * @attention
  *
  * 本文件属于 apple_detect_task ROS 2 功能包。
  ******************************************************************************
  */

/* Includes -----------------------------------------------------------*/

#include <memory>

#include "apple_detect_task/image_publisher.hpp"
#include "rclcpp/rclcpp.hpp"

/* Main Function ------------------------------------------------------*/

/**
 * @brief 图像发布节点程序入口。
 *
 * 负责初始化 ROS 2、创建 ImagePublisher 节点、进入事件循环，
 * 并在程序结束时安全关闭 ROS 2。
 *
 * @param argc 命令行参数数量。
 * @param argv 命令行参数数组。
 * @return 程序退出状态，0 表示正常退出。
 */
int main(int argc, char * argv[])
{
  // 初始化 ROS 2 通信环境，同时解析命令行参数。
  rclcpp::init(argc, argv);

  // 创建图像发布节点，并进入 ROS 2 事件循环。
  rclcpp::spin(std::make_shared<ImagePublisher>());

  // 停止 ROS 2 通信并释放相关资源。
  rclcpp::shutdown();

  return 0;
}
