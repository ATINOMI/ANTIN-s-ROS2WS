/* Includes ----------------------------------------------------------------*/
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "costmap_publisher.hpp"

namespace
{
  constexpr unsigned char kLethalObstacle = 254;
}


/* Application entry point -------------------------------------------------*/
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  /* Node configuration ------------------------------------------------------*/
  auto node = std::make_shared<mini_nav_nodes::CostmapPublisherNode>(
     40,  // size_x：x 方向 40 格
     30,  // size_y：y 方向 30 格
     0.1,  // resolution：每格 1 m
     0,  // origin_x
     0,  // origin_y 
     0,    // default_value：空闲
    1000); // publish_period_ms：每秒发布一次

  /* Maze layout -------------------------------------------------------------*/
  auto & costmap = node->GetCostmap();

  // 先封闭地图边界。x 合法范围是 0~399，y 合法范围是 0~299。
  costmap.DrawLine({0, 0}, {0, 29}, kLethalObstacle);
  costmap.DrawLine({39, 0}, {39, 29}, kLethalObstacle);
  costmap.DrawLine({0, 0}, {39, 0}, kLethalObstacle);
  costmap.DrawLine({0, 29}, {39, 29}, kLethalObstacle);

  // 内部迷宫墙：端点坐标按 10 倍放大，保留原有布局比例。
  costmap.DrawLine({7, 0}, {10, 10}, kLethalObstacle);
  costmap.DrawLine({7, 20}, {16, 20}, kLethalObstacle);
  costmap.DrawLine({16, 15}, {16, 29}, kLethalObstacle);
  costmap.DrawLine({16, 8}, {25, 8}, kLethalObstacle);
  costmap.DrawLine({25, 0}, {25, 12}, kLethalObstacle);
  costmap.DrawLine({25, 18}, {34, 18}, kLethalObstacle);
  costmap.DrawLine({34, 9}, {34, 29}, kLethalObstacle);

  /* Node execution ----------------------------------------------------------*/
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}