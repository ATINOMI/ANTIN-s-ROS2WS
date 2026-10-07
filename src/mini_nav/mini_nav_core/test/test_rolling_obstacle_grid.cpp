/**
 * @file test_rolling_obstacle_grid.cpp
 * @brief 验证 rolling_obstacle_grid 模块行为及边界的测试。
 * @author Antinomy
 * @date 2026-10-01
 */
#include <gtest/gtest.h>

#include "mini_nav_core/map/rolling_obstacle_grid.hpp"

namespace
{
    /**
     * @brief 按米制坐标查询测试地图代价，同时断言坐标在图内。
     *
     * @param map 被查询的测试图。
     * @param x 世界 x，米。
     * @param y 世界 y，米。
     * @return 所在格的内部代价。
     */
    unsigned char CostAt(const mini_nav_core::Costmap2D & map, double x, double y)
    {
        unsigned int mx = 0;
        unsigned int my = 0;
        EXPECT_TRUE(map.WorldToMap(x, y, mx, my));
        return map.GetCost(mx, my);
    }
}

/**
 * @brief 验证射线清空、端点标障及膨胀。
 */
TEST(RollingObstacleGrid, MarksClearsAndInflatesScanEndpoint)
{
    mini_nav_core::InflationParameters inflation;
    inflation.robot_radius = 0.1;
    inflation.safety_margin = 0.05;
    inflation.inflation_radius = 0.3;
    mini_nav_core::RollingObstacleGrid grid(20, 20, 0.1, inflation);
    grid.CenterOn(0.0, 0.0);

    ASSERT_TRUE(grid.IntegrateRay(0.0, 0.0, 0.5, 0.0, 1.0));
    ASSERT_TRUE(grid.MarkObstacle(0.5, 0.0, 1.0));
    EXPECT_EQ(CostAt(grid.Raw(), 0.0, 0.0), 0);
    EXPECT_EQ(CostAt(grid.Raw(), 0.5, 0.0), 254);
    EXPECT_EQ(CostAt(grid.Inflated(), 0.45, 0.05), 253);

    ASSERT_TRUE(grid.IntegrateRay(0.0, 0.0, 0.7, 0.0, 2.0));
    EXPECT_EQ(CostAt(grid.Raw(), 0.5, 0.0), 0);
    EXPECT_EQ(CostAt(grid.Inflated(), 0.45, 0.05), 0);
}

/**
 * @brief 验证窗口滚动保留重叠观测，过期后恢复未知。
 */
TEST(RollingObstacleGrid, PreservesOverlapAndResetsExpiredObservations)
{
    mini_nav_core::RollingObstacleGrid grid(20, 20, 0.1, {});
    grid.CenterOn(0.0, 0.0);
    ASSERT_TRUE(grid.MarkObstacle(0.5, 0.0, 1.0));
    grid.CenterOn(0.2, 0.0);

    EXPECT_NEAR(grid.Raw().GetOriginX(), -0.8, 1.0e-9);
    EXPECT_EQ(CostAt(grid.Raw(), 0.5, 0.0), 254);
    EXPECT_EQ(CostAt(grid.Raw(), 1.1, 0.0), 255);

    grid.Expire(1.5, 1.0);
    EXPECT_EQ(CostAt(grid.Raw(), 0.5, 0.0), 254);
    grid.Reset();
    EXPECT_EQ(CostAt(grid.Raw(), 0.5, 0.0), 255);
    EXPECT_FALSE(grid.IntegrateRay(5.0, 0.0, 5.5, 0.0, 2.0));
}

/**
 * @brief 验证正反整格边界浮点误差不妨碍滚动。
 */
TEST(RollingObstacleGrid, RollsAtWholeCellBoundaryInBothDirections)
{
    mini_nav_core::RollingObstacleGrid grid(20, 20, 0.1, {});
    grid.CenterOn(0.0, 0.0);
    ASSERT_TRUE(grid.MarkObstacle(0.55, 0.05, 1.0));

    grid.CenterOn(0.09, 0.0);
    EXPECT_NEAR(grid.Raw().GetOriginX(), -1.0, 1.0e-9);

    grid.CenterOn(0.1, 0.0);
    EXPECT_NEAR(grid.Raw().GetOriginX(), -0.9, 1.0e-9);
    EXPECT_EQ(CostAt(grid.Raw(), 0.55, 0.05), 254);

    grid.CenterOn(-0.1, 0.0);
    EXPECT_NEAR(grid.Raw().GetOriginX(), -1.1, 1.0e-9);
    EXPECT_EQ(CostAt(grid.Raw(), 0.55, 0.05), 254);
}

/**
 * @brief 验证按每格观测时间独立过期。
 */
TEST(RollingObstacleGrid, ExpiresIndividualObservations)
{
    mini_nav_core::RollingObstacleGrid grid(20, 20, 0.1, {});
    grid.CenterOn(0.0, 0.0);
    ASSERT_TRUE(grid.MarkObstacle(0.5, 0.0, 1.0));
    ASSERT_TRUE(grid.IntegrateRay(0.0, 0.0, 0.2, 0.0, 2.0));

    grid.Expire(2.5, 1.0);
    EXPECT_EQ(CostAt(grid.Raw(), 0.5, 0.0), 255);
    EXPECT_EQ(CostAt(grid.Raw(), 0.1, 0.0), 0);

    grid.Expire(3.1, 1.0);
    EXPECT_EQ(CostAt(grid.Raw(), 0.1, 0.0), 255);
}

/**
 * @brief 验证角点射线不清除旁侧未观测格。
 */
TEST(RollingObstacleGrid, DiagonalRayKeepsCellsBesideGridCorner)
{
    mini_nav_core::RollingObstacleGrid grid(20, 20, 0.1, {});
    grid.CenterOn(0.0, 0.0);
    ASSERT_TRUE(grid.MarkObstacle(0.15, 0.05, 1.0));
    ASSERT_TRUE(grid.MarkObstacle(0.05, 0.15, 1.0));

    ASSERT_TRUE(grid.IntegrateRay(0.05, 0.05, 0.25, 0.25, 2.0));
    EXPECT_EQ(CostAt(grid.Raw(), 0.15, 0.05), 254);
    EXPECT_EQ(CostAt(grid.Raw(), 0.05, 0.15), 254);
    EXPECT_EQ(CostAt(grid.Raw(), 0.15, 0.15), 0);
}
