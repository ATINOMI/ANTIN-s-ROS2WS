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
