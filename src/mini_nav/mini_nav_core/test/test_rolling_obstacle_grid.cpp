#include <gtest/gtest.h>

#include "mini_nav_core/map/rolling_obstacle_grid.hpp"

namespace
{
    unsigned char CostAt(const mini_nav_core::Costmap2D & map, double x, double y)
    {
        unsigned int mx = 0;
        unsigned int my = 0;
        EXPECT_TRUE(map.WorldToMap(x, y, mx, my));
        return map.GetCost(mx, my);
    }
}

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
