/**
 * @file test_astar_planner.cpp
 * @brief 验证 astar_planner 模块行为及边界的测试。
 * @author Antinomy
 * @date 2026-10-01
 */
/* Includes ----------------------------------------------------------------*/
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "mini_nav_core/navigator/path_postprocessor.hpp"
#include "mini_nav_core/navigator/path_tracker.hpp"

#include "mini_nav_core/navigator/astar_navigator.hpp"

/* Type aliases ------------------------------------------------------------*/
using mini_nav_core::AStarPlanner;
using mini_nav_core::Costmap2D;
using mini_nav_core::MapLocation;

/* Test cases --------------------------------------------------------------*/
/**
 * @brief 验证 A* 绕开障碍墙并连接起终点。
 */
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


/**
 * @brief 验证原目标可达时优先返回原目标。
 */
TEST(AStarPlanner, PrefersExactGoalWhenReachable)
{
    Costmap2D map(7, 5, 0.25, 0.0, 0.0);
    AStarPlanner planner;
    const auto path = planner.Plan(map, {1, 2}, {5, 2}, 0.5);
    ASSERT_FALSE(path.empty());
    EXPECT_EQ(path.back().x, 5u);
    EXPECT_EQ(path.back().y, 2u);
}

/**
 * @brief 验证障碍目标替换为容差内最近可达安全格。
 */
TEST(AStarPlanner, ReplacesBlockedGoalWithNearestReachableCell)
{
    for (const unsigned char cost : {253, 254, 255}) {
        Costmap2D map(7, 7, 0.25, 0.0, 0.0);
        map.SetCost(3, 3, cost);
        AStarPlanner planner;
        const auto path = planner.Plan(map, {1, 3}, {3, 3}, 0.25);
        ASSERT_FALSE(path.empty()) << "goal cost " << static_cast<int>(cost);
        // 等距离候选中，选从起点到达代价较低的格子。
        EXPECT_EQ(path.back().x, 2u);
        EXPECT_EQ(path.back().y, 3u);
        for (const auto & cell : path) {
            EXPECT_LT(map.GetCost(cell.x, cell.y), 253);
        }
    }
}

/**
 * @brief 验证替代终点位于起点可达的连通区域。
 */
TEST(AStarPlanner, ChoosesReachableSideOfDisconnectedGoal)
{
    Costmap2D map(7, 5, 0.25, 0.0, 0.0);
    map.DrawLine({3, 0}, {3, 4}, 254);
    AStarPlanner planner;
    // 目标侧存在更近的自由格，但它们都不能从起点到达。
    const auto path = planner.Plan(map, {0, 2}, {5, 2}, 0.75);
    ASSERT_FALSE(path.empty());
    EXPECT_EQ(path.back().x, 2u);
    EXPECT_EQ(path.back().y, 2u);
    EXPECT_TRUE(planner.Plan(map, {0, 2}, {5, 2}, 0.74).empty());
}

/**
 * @brief 验证容差范围没有可达格时返回无路。
 */
TEST(AStarPlanner, FailsWhenNoReachableCellWithinTolerance)
{
    Costmap2D map(7, 7, 0.25, 0.0, 0.0);
    map.SetCost(3, 3, 254);
    AStarPlanner planner;
    EXPECT_TRUE(planner.Plan(map, {1, 3}, {3, 3}, 0.24).empty());
    EXPECT_TRUE(planner.Plan(map, {1, 3}, {3, 3}, 0.0).empty());
    EXPECT_TRUE(planner.Plan(map, {1, 3}, {3, 3}).empty());
}

/**
 * @brief 验证目标容差不放宽起点安全与地图边界。
 */
TEST(AStarPlanner, DoesNotRelaxInvalidStartOrMapBounds)
{
    Costmap2D map(7, 7, 0.25, 0.0, 0.0);
    map.SetCost(1, 3, 253);
    AStarPlanner planner;
    EXPECT_TRUE(planner.Plan(map, {1, 3}, {3, 3}, 1.0).empty());
    EXPECT_TRUE(planner.Plan(map, {7, 3}, {3, 3}, 1.0).empty());
    EXPECT_TRUE(planner.Plan(map, {2, 3}, {7, 3}, 1.0).empty());
    const auto path = planner.Plan(map, {2, 3}, {2, 3}, 1.0);
    ASSERT_EQ(path.size(), 1u);
    EXPECT_EQ(path.back().x, 2u);
    EXPECT_EQ(path.back().y, 3u);
}

/**
 * @brief 验证负值及非有限目标容差被拒绝。
 */
TEST(AStarPlanner, RejectsInvalidGoalTolerance)
{
    Costmap2D map(7, 7, 0.25, 0.0, 0.0);
    AStarPlanner planner;
    for (const double tolerance : {-1.0, std::numeric_limits<double>::infinity(),
                                  std::numeric_limits<double>::quiet_NaN()}) {
        EXPECT_THROW(planner.Plan(map, {1, 3}, {3, 3}, tolerance), std::invalid_argument);
        EXPECT_THROW(planner.Plan(map, map, 0.1, {1, 3}, {3, 3}, true, tolerance),
                     std::invalid_argument);
    }
}

/**
 * @brief 验证后备终点仍满足图边界车体连续余量。
 */
TEST(AStarPlanner, FallbackRespectsContinuousBodyClearanceAtBoundary)
{
    Costmap2D map(7, 7, 1.0, 0.0, 0.0);
    AStarPlanner planner;
    EXPECT_TRUE(planner.Plan(map, map, 0.6, {3, 3}, {0, 3}, true).empty());
    const auto path = planner.Plan(map, map, 0.6, {3, 3}, {0, 3}, true, 1.0);
    ASSERT_FALSE(path.empty());
    EXPECT_EQ(path.back().x, 1u);
    EXPECT_EQ(path.back().y, 3u);
    // 容差也不能放宽起点的车体越界条件。
    EXPECT_TRUE(planner.Plan(map, map, 0.6, {0, 3}, {3, 3}, true, 1.0).empty());
}

/**
 * @brief 验证格中心空闲不能代替完整车体安全。
 */
TEST(AStarPlanner, FallbackChecksEntireBodyRatherThanFreeCellCenter)
{
    Costmap2D source(7, 7, 1.0, 0.0, 0.0);
    source.SetCost(3, 3, 254);
    AStarPlanner planner;
    const auto path = planner.Plan(source, source, 0.6, {1, 3}, {3, 3}, true, 1.5);
    ASSERT_FALSE(path.empty());
    // 紧邻障碍的自由格中心只有 0.5 m 余量，不能容纳 0.6 m 圆盘。
    EXPECT_NEAR(std::hypot(static_cast<double>(path.back().x) - 3.0,
                           static_cast<double>(path.back().y) - 3.0), std::sqrt(2.0), 1e-9);
    for (std::size_t i = 1; i < path.size(); ++i) {
        mini_nav_core::PathPoint from{}, to{};
        source.MapToWorld(path[i - 1].x, path[i - 1].y, from.x, from.y);
        source.MapToWorld(path[i].x, path[i].y, to.x, to.y);
        EXPECT_TRUE(mini_nav_core::IsCircularSweepClear(source, from, to, 0.6, true));
    }
}


/**
 * @brief 验证替代终点能由跟踪器判断到达并停车。
 */
TEST(AStarPlanner, ReplacementEndpointIsAcceptedAsReachedByTracker)
{
    Costmap2D map(40, 40, 0.1, 0.0, 0.0);
    map.SetCost(20, 20, 254);
    AStarPlanner planner;
    const auto cells = planner.Plan(map, map, 0.26, {10, 20}, {20, 20}, true, 0.5);
    ASSERT_FALSE(cells.empty());
    EXPECT_FALSE(cells.back().x == 20u && cells.back().y == 20u);
    std::vector<mini_nav_core::PathPoint> points;
    for (const auto & cell : cells) {
        mini_nav_core::PathPoint point{};
        map.MapToWorld(cell.x, cell.y, point.x, point.y);
        points.push_back(point);
    }
    mini_nav_core::PathTrackerParameters parameters;
    parameters.max_linear_speed = 0.15;
    parameters.max_angular_speed = 0.55;
    mini_nav_core::PathTracker tracker(parameters);
    tracker.SetPath(points, 0.7);
    const mini_nav_core::localization::Pose2D pose{points.back().x, points.back().y, 0.7};
    const auto command = tracker.Step(pose, pose, {0.0, 0.0, 0.0}, map, map, 0.0);
    EXPECT_EQ(command.status, mini_nav_core::TrackingStatus::kGoalReached);
    EXPECT_DOUBLE_EQ(command.linear_x, 0.0);
    EXPECT_DOUBLE_EQ(command.angular_z, 0.0);
}
