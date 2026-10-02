/**
 * @file main.cpp
 * @brief 共享 ROS 进程入口，通过编译宏选择节点。
 * @author Antinomy
 * @date 2026-10-01
 */
#include <memory>

#include "rclcpp/rclcpp.hpp"
#ifdef MINI_NAV_BUILD_NAVIGATION_MANAGER
#include "mini_nav_nodes/navigation_manager_node.hpp"
#elif defined(MINI_NAV_BUILD_VELOCITY_GUARD)
#include "mini_nav_nodes/velocity_guard_node.hpp"
#elif defined(MINI_NAV_BUILD_AMCL)
#include "mini_nav_nodes/amcl_node.hpp"
#elif defined(MINI_NAV_BUILD_LOCAL_COSTMAP)
#include "mini_nav_nodes/local_costmap_node.hpp"
#elif defined(MINI_NAV_BUILD_PATH_FOLLOWER)
#include "mini_nav_nodes/path_follower_node.hpp"
#else
#include "costmap_publisher.hpp"
#endif

/**
 * @brief 初始化 ROS 并按编译宏启动对应节点，spin 结束后关闭 context。
 *
 * 各可执行目标复用入口；业务行为在节点类中，生命周期 AMCL 使用其 base interface。
 *
 * @param argc 命令行参数数量。
 * @param argv 参数数组，由 rclcpp 解析 ROS 参数。
 * @return 正常退出返回 0；未捕获异常由运行时终止进程。
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

#ifdef MINI_NAV_BUILD_NAVIGATION_MANAGER
  rclcpp::spin(std::make_shared<mini_nav_nodes::NavigationManagerNode>());
#elif defined(MINI_NAV_BUILD_VELOCITY_GUARD)
  rclcpp::spin(std::make_shared<mini_nav_nodes::VelocityGuardNode>());
#elif defined(MINI_NAV_BUILD_AMCL)
  auto node = std::make_shared<mini_nav_nodes::AmclNode>();
  rclcpp::spin(node->get_node_base_interface());
#elif defined(MINI_NAV_BUILD_LOCAL_COSTMAP)
  auto node = std::make_shared<mini_nav_nodes::LocalCostmapNode>();
  rclcpp::spin(node);
#elif defined(MINI_NAV_BUILD_PATH_FOLLOWER)
  auto node = std::make_shared<mini_nav_nodes::PathFollowerNode>();
  rclcpp::spin(node);
#else
  auto node = std::make_shared<mini_nav_nodes::CostmapPublisherNode>(
    40, 30, 0.1, 0.0, 0.0, 0, 1000);

  rclcpp::spin(node);
#endif
  rclcpp::shutdown();
  return 0;
}
