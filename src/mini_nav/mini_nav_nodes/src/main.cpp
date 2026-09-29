#include <memory>

#include "rclcpp/rclcpp.hpp"
#ifdef MINI_NAV_BUILD_AMCL
#include "mini_nav_nodes/amcl_node.hpp"
#elif defined(MINI_NAV_BUILD_LOCAL_COSTMAP)
#include "mini_nav_nodes/local_costmap_node.hpp"
#else
#include "costmap_publisher.hpp"
#endif

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

#ifdef MINI_NAV_BUILD_AMCL
  auto node = std::make_shared<mini_nav_nodes::AmclNode>();
  rclcpp::spin(node->get_node_base_interface());
#elif defined(MINI_NAV_BUILD_LOCAL_COSTMAP)
  auto node = std::make_shared<mini_nav_nodes::LocalCostmapNode>();
  rclcpp::spin(node);
#else
  auto node = std::make_shared<mini_nav_nodes::CostmapPublisherNode>(
    40, 30, 0.1, 0.0, 0.0, 0, 1000);

  rclcpp::spin(node);
#endif
  rclcpp::shutdown();
  return 0;
}
