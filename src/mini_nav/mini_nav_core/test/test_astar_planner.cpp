/* Includes ----------------------------------------------------------------*/
#include <gtest/gtest.h>

#include "mini_nav_core/astar_navigator.hpp"

/* Type aliases ------------------------------------------------------------*/
using mini_nav_core::AStarPlanner;
using mini_nav_core::Costmap2D;
using mini_nav_core::MapLocation;

/* Test cases --------------------------------------------------------------*/
TEST(AStarPlanner, FindsPathAroundWall)
{
  Costmap2D map(7, 5, 1.0, 0.0, 0.0);

  // 在 x=3 放一堵不封顶的墙，路径只能从最上面一行绕过。
  map.DrawLine({3, 0}, {3, 3}, 254);

  AStarPlanner planner;
  const MapLocation start{1, 1};
  const MapLocation goal{5, 1};
  const std::vector<MapLocation> path = planner.Plan(map, start, goal);

  ASSERT_FALSE(path.empty());
  EXPECT_EQ(path.front().x, start.x);
  EXPECT_EQ(path.front().y, start.y);
  EXPECT_EQ(path.back().x, goal.x);
  EXPECT_EQ(path.back().y, goal.y);

  bool reaches_top_row = false;
  for (const auto & cell : path) {
    EXPECT_LT(map.GetCost(cell.x, cell.y), 254);
    reaches_top_row = reaches_top_row || cell.y == 4;
  }
  EXPECT_TRUE(reaches_top_row);
}
