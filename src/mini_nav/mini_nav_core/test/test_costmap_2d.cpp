/* Includes ----------------------------------------------------------------*/
#include <gtest/gtest.h>

#include "mini_nav_core/costmap_2d.hpp"

/* Type aliases ------------------------------------------------------------*/
using mini_nav_core::Costmap2D;

/* Test cases --------------------------------------------------------------*/
// 验证构造函数是否保存地图元数据，并用默认代价填充全部栅格。
TEST(Costmap2D, ConstructorInitializesMetadataAndCells)
{
  // 地图：4 × 3 格，每格 0.5 m，世界坐标原点为 (-1, -2)。
  Costmap2D map(4, 3, 0.5, -1.0, -2.0, 7);

  // 先检查地图元数据。
  EXPECT_EQ(map.GetSizeInCellsX(), 4u);
  EXPECT_EQ(map.GetSizeInCellsY(), 3u);
  EXPECT_DOUBLE_EQ(map.GetResolution(), 0.5);
  EXPECT_DOUBLE_EQ(map.GetOriginX(), -1.0);
  EXPECT_DOUBLE_EQ(map.GetOriginY(), -2.0);

  // 再检查角落格子都已被初始化为默认代价 7。
  EXPECT_EQ(map.GetCost(0, 0), 7u);
  EXPECT_EQ(map.GetCost(3, 2), 7u);
}

// 验证写入一个格子不会意外修改其他格子。
TEST(Costmap2D, SetCostChangesOnlyTargetCell)
{
  Costmap2D map(4, 3, 0.5, -1.0, -2.0, 7);

  map.SetCost(2, 1, 254);

  // (2, 1) 的一维下标为 1 * 4 + 2 = 6。
  EXPECT_EQ(map.GetCost(2, 1), 254u);
  EXPECT_EQ(map.GetCost(6), 254u);

  // 相邻格子仍应保持默认值。
  EXPECT_EQ(map.GetCost(1, 1), 7u);
}

// 验证格子坐标转换为该格子的世界坐标中心点。
TEST(Costmap2D, MapToWorldReturnsCellCenter)
{
  Costmap2D map(4, 3, 0.5, -1.0, -2.0);
  double wx = 0.0;
  double wy = 0.0;

  map.MapToWorld(2, 1, wx, wy);

  // wx = -1 + (2 + 0.5) * 0.5，wy = -2 + (1 + 0.5) * 0.5。
  EXPECT_DOUBLE_EQ(wx, 0.25);
  EXPECT_DOUBLE_EQ(wy, -1.25);
}

// 验证世界坐标到地图坐标的转换，以及地图边界判定。
TEST(Costmap2D, WorldToMapConvertsCoordinatesAndChecksBounds)
{
  Costmap2D map(4, 3, 0.5, -1.0, -2.0);
  unsigned int mx = 0;
  unsigned int my = 0;

  EXPECT_TRUE(map.WorldToMap(0.0, -1.5, mx, my));
  EXPECT_EQ(mx, 2u);
  EXPECT_EQ(my, 1u);

  // 左边界之外、右边界和上边界本身都不是合法栅格。
  EXPECT_FALSE(map.WorldToMap(-1.01, -2.0, mx, my));
  EXPECT_FALSE(map.WorldToMap(1.0, -2.0, mx, my));
  EXPECT_FALSE(map.WorldToMap(0.0, -0.5, mx, my));
}

// 验证 ResizeMap 会替换旧地图，并把所有新栅格恢复为默认代价。
TEST(Costmap2D, ResizeMapResetsCellsToDefaultValue)
{
  Costmap2D map(3, 2, 0.5, -1.0, -2.0, 7);
  map.SetCost(1, 1, 254);

  map.ResizeMap(2, 4, 1.0, 10.0, -3.0);

  EXPECT_EQ(map.GetSizeInCellsX(), 2u);
  EXPECT_EQ(map.GetSizeInCellsY(), 4u);
  EXPECT_DOUBLE_EQ(map.GetResolution(), 1.0);
  EXPECT_DOUBLE_EQ(map.GetOriginX(), 10.0); 
  EXPECT_DOUBLE_EQ(map.GetOriginY(), -3.0);

  // 遍历新地图，确认旧的障碍物代价没有被保留。
  for (unsigned int my = 0; my < map.GetSizeInCellsY(); ++my) {
    for (unsigned int mx = 0; mx < map.GetSizeInCellsX(); ++mx) {
      EXPECT_EQ(map.GetCost(mx, my), 7u);
    }
  }
}
