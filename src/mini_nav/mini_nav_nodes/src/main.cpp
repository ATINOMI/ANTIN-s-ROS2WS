/* Includes ----------------------------------------------------------------*/
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "costmap_publisher.hpp"


/* Application entry point -------------------------------------------------*/
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  /* Node configuration ------------------------------------------------------*/
  auto node = std::make_shared<mini_nav_nodes::CostmapPublisherNode>(
     40,   // size_x：x 方向 40 格
     30,   // size_y：y 方向 30 格
     0.1,  // resolution：每格 0.1 m
     0,  // origin_x
     0,  // origin_y 
     0,    // default_value：空闲
    1000); // publish_period_ms：每秒发布一次

  /* Maze layout -------------------------------------------------------------*/
  // 先封闭地图边界。x 合法范围是 0~39，y 合法范围是 0~29。
  node->addWall(0, 0, 30);          // 左边界：向 +y
  node->addWall(39, 0, 30);         // 右边界：向 +y
  node->addWall(0, 0, 40, false);   // 下边界：向 +x
  node->addWall(0, 29, 40, false);  // 上边界：向 +x

  // 内部迷宫墙：每一段都留出通道，起点和终点不会被完全隔开。
  node->addWall(7, 1, 15);          // 竖墙，顶部留通道
  node->addWall(7, 20, 10, false);  // 横墙，向右
  node->addWall(16, 15, 14);         // 竖墙，底部留通道
  node->addWall(16, 8, 9, false);   // 横墙，向右
  node->addWall(25, 1, 12);         // 竖墙，顶部留通道
  node->addWall(25, 18, 10, false); // 横墙，向右
  node->addWall(34, 9, 20);         // 竖墙，底部留通道

  /* Node execution ----------------------------------------------------------*/
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}