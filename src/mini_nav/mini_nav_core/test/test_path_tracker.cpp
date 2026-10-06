/**
 * @file test_path_tracker.cpp
 * @brief 验证 path_tracker 模块行为及边界的测试。
 * @author Antinomy
 * @date 2026-10-01
 */
#include <gtest/gtest.h>

#include <limits>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>

#include "mini_nav_core/navigator/controller/path_tracker.hpp"

namespace
{
    /**
     * @brief 建立测试用的全空闲地图。
     * @return 100×100 格、0.05 m 分辨率、原点 (-2.5,-2.5) 的独立地图。
     */
    mini_nav_core::Costmap2D ClearMap()
    {
        return mini_nav_core::Costmap2D(100, 100, 0.05, -2.5, -2.5, 0);
    }

    /**
     * @brief 构造测试二维位姿。
     *
     * @param x 位置 x，米。
     * @param y 位置 y，米。
     * @param yaw 偏航，弧度。
     * @return 包含输入分量的位姿。
     */
    mini_nav_core::nav_types::Pose2D Pose(double x, double y, double yaw = 0.0)
    {
        return {x, y, yaw};
    }

    /**
     * @brief 在 map 与 odom 重合的测试场景执行一步跟踪。
     *
     * @param tracker 被测跟踪器。
     * @param static_map 全局安全图。
     * @param local_map 局部安全图。
     * @param pose 两系共享的机器人位姿。
     * @param seconds 测试稳态时间，秒。
     * @return 跟踪速度与状态。
     */
    mini_nav_core::VelocityCommand Step(
        mini_nav_core::PathTracker & tracker,
        const mini_nav_core::Costmap2D & static_map,
        const mini_nav_core::Costmap2D & local_map,
        mini_nav_core::nav_types::Pose2D pose,
        double seconds)
    {
        return tracker.Step(pose, pose, Pose(0.0, 0.0), static_map, local_map, seconds);
    }
}

/**
 * @brief 验证直线路径跟踪与线/角速度上限。
 */
TEST(PathTracker, TracksStraightPathWithSpeedLimits)
{
    mini_nav_core::PathTracker tracker;
    const auto map = ClearMap();
    tracker.SetPath({{0.0, 0.0}, {1.0, 0.0}}, 0.0);

    const auto forward = Step(tracker, map, map, Pose(0.0, 0.0), 0.0);
    EXPECT_EQ(forward.status, mini_nav_core::TrackingStatus::kTracking);
    EXPECT_GT(forward.linear_x, 0.0);
    EXPECT_LE(forward.linear_x, 0.10);
    EXPECT_DOUBLE_EQ(forward.angular_z, 0.0);

    tracker.PauseProgress();
    const auto turn = Step(tracker, map, map, Pose(0.0, 0.0, 1.0), 1.0);
    EXPECT_DOUBLE_EQ(turn.linear_x, 0.0);
    EXPECT_GE(turn.angular_z, -0.40);
    EXPECT_LT(turn.angular_z, 0.0);
}

/**
 * @brief 验证末点转向、到达锁存与同一路径重发后停车。
 */
TEST(PathTracker, RotatesAtGoalThenLatchesStopAcrossPathRepublication)
{
    mini_nav_core::PathTracker tracker;
    const auto map = ClearMap();
    const std::vector<mini_nav_core::PathPoint> path{{0.0, 0.0}, {1.0, 0.0}};
    tracker.SetPath(path, 1.0);

    const auto turn = Step(tracker, map, map, Pose(1.0, 0.0, 0.0), 0.0);
    EXPECT_DOUBLE_EQ(turn.linear_x, 0.0);
    EXPECT_GT(turn.angular_z, 0.0);
    EXPECT_LE(turn.angular_z, 0.40);

    const auto reached = Step(tracker, map, map, Pose(1.0, 0.0, 0.95), 1.0);
    EXPECT_EQ(reached.status, mini_nav_core::TrackingStatus::kGoalReached);
    EXPECT_DOUBLE_EQ(reached.linear_x, 0.0);
    EXPECT_DOUBLE_EQ(reached.angular_z, 0.0);

    tracker.SetPath(path, 1.0);
    const auto repeated = Step(tracker, map, map, Pose(0.5, 0.0), 2.0);
    EXPECT_EQ(repeated.status, mini_nav_core::TrackingStatus::kGoalReached);
    EXPECT_DOUBLE_EQ(repeated.linear_x, 0.0);
}

/**
 * @brief 终点内圈进入转向，定位轻微波动时保持转向，外圈仍按原容差判定到达。
 */
TEST(PathTracker, GoalPositionHysteresisKeepsHeadingWithoutRelaxingArrival)
{
    mini_nav_core::PathTrackerParameters parameters;
    parameters.goal_position_hysteresis = 0.03;
    mini_nav_core::PathTracker tracker(parameters);
    const auto map = ClearMap();
    tracker.SetPath({{0.0, 0.0}, {1.0, 0.0}}, 1.0);

    EXPECT_GT(Step(tracker, map, map, Pose(0.881, 0.0), 0.0).linear_x, 0.0);
    EXPECT_DOUBLE_EQ(Step(tracker, map, map, Pose(0.915, 0.0), 1.0).linear_x, 0.0);
    const auto noisy = Step(tracker, map, map, Pose(0.885, 0.0), 2.0);
    EXPECT_DOUBLE_EQ(noisy.linear_x, 0.0);
    EXPECT_GT(noisy.angular_z, 0.0);
    EXPECT_GT(Step(tracker, map, map, Pose(0.879, 0.0), 3.0).linear_x, 0.0);
    EXPECT_NE(Step(tracker, map, map, Pose(0.879, 0.0, 1.0), 4.0).status,
              mini_nav_core::TrackingStatus::kGoalReached);
    EXPECT_EQ(Step(tracker, map, map, Pose(0.885, 0.0, 1.0), 5.0).status,
              mini_nav_core::TrackingStatus::kGoalReached);

    tracker.ClearPath();
    tracker.SetPath({{0.0, 0.0}, {1.0, 0.0}}, 1.0);
    EXPECT_GT(Step(tracker, map, map, Pose(0.881, 0.0), 6.0).linear_x, 0.0);
}

TEST(PathTracker, RejectsInvalidGoalPositionHysteresis)
{
    mini_nav_core::PathTrackerParameters parameters;
    for (const double value : {-0.01, 0.12, std::numeric_limits<double>::infinity()}) {
        parameters.goal_position_hysteresis = value;
        EXPECT_THROW({ mini_nav_core::PathTracker tracker(parameters); }, std::invalid_argument);
    }
}

/**
 * @brief 验证全局、局部障碍及未知格导致停车。
 */
TEST(PathTracker, StopsForStaticLocalAndUnknownCells)
{
    mini_nav_core::PathTracker tracker;
    tracker.SetPath({{0.0, 0.0}, {1.0, 0.0}}, 0.0);
    auto static_map = ClearMap();
    auto local_map = ClearMap();
    static_map.SetCost(51, 50, 253);
    EXPECT_EQ(Step(tracker, static_map, local_map, Pose(0.0, 0.0), 0.0).status,
              mini_nav_core::TrackingStatus::kCollisionRisk);

    static_map.SetCost(51, 50, 0);
    local_map.SetCost(51, 50, 254);
    EXPECT_EQ(Step(tracker, static_map, local_map, Pose(0.0, 0.0), 1.0).status,
              mini_nav_core::TrackingStatus::kCollisionRisk);

    local_map.SetCost(51, 50, 255);
    EXPECT_EQ(Step(tracker, static_map, local_map, Pose(0.0, 0.0), 2.0).status,
              mini_nav_core::TrackingStatus::kCollisionRisk);
}

/**
 * @brief 验证无进展超时锁存，须新路径恢复。
 */
TEST(PathTracker, StopsAfterNoProgressAndRequiresNewPath)
{
    mini_nav_core::PathTracker tracker;
    const auto map = ClearMap();
    const std::vector<mini_nav_core::PathPoint> path{{0.0, 0.0}, {1.0, 0.0}};
    tracker.SetPath(path, 0.0);
    EXPECT_GT(Step(tracker, map, map, Pose(0.0, 0.0), 0.0).linear_x, 0.0);

    const auto timed_out = Step(tracker, map, map, Pose(0.0, 0.0), 11.0);
    EXPECT_EQ(timed_out.status, mini_nav_core::TrackingStatus::kProgressTimeout);
    EXPECT_DOUBLE_EQ(timed_out.linear_x, 0.0);

    tracker.SetPath(path, 0.0);
    EXPECT_EQ(Step(tracker, map, map, Pose(0.0, 0.0), 12.0).status,
              mini_nav_core::TrackingStatus::kProgressTimeout);

    tracker.ClearPath();
    EXPECT_EQ(Step(tracker, map, map, Pose(0.0, 0.0), 13.0).status,
              mini_nav_core::TrackingStatus::kNoPath);
    tracker.SetPath(path, 0.0);
    EXPECT_GT(Step(tracker, map, map, Pose(0.0, 0.0), 14.0).linear_x, 0.0);
}

/**
 * @brief 验证偏离路径及非有限位姿输出零速。
 */
TEST(PathTracker, RejectsDistantPathAndInvalidPose)
{
    mini_nav_core::PathTracker tracker;
    const auto map = ClearMap();
    tracker.SetPath({{1.0, 1.0}, {2.0, 1.0}}, 0.0);
    EXPECT_EQ(Step(tracker, map, map, Pose(0.0, 0.0), 0.0).status,
              mini_nav_core::TrackingStatus::kOffPath);
    tracker.SetPath({{0.0, 0.0}, {1.0, 0.0}}, 0.0);
    EXPECT_EQ(Step(tracker, map, map, Pose(0.0, 0.0),
                   std::numeric_limits<double>::quiet_NaN()).status,
              mini_nav_core::TrackingStatus::kInvalidPose);
}


/**
 * @brief 验证采集快照中接近终点的局部边界不会误停。
 */
TEST(PathTracker, CapturedLocalGoalBoundaryAllowsSafeApproach)
{
    const auto filename = std::filesystem::path(__FILE__).parent_path() /
        "data/path_tracker_collision_snapshot.txt";
    std::ifstream input(filename);
    ASSERT_TRUE(input.is_open());
    int samples = 0;
    input >> samples;
    ASSERT_EQ(samples, 1);
    const auto read_map = [&]() {
        unsigned int width = 0, height = 0;
        double resolution = 0.0, origin_x = 0.0, origin_y = 0.0;
        input >> width >> height >> resolution >> origin_x >> origin_y;
        mini_nav_core::Costmap2D map(width, height, resolution, origin_x, origin_y, 0);
        for (unsigned int y = 0; y < height; ++y) {
            for (unsigned int x = 0; x < width; ++x) {
                int cost = 0;
                if (!(input >> cost)) throw std::runtime_error("Incomplete collision snapshot");
                map.SetCost(x, y, static_cast<unsigned char>(cost));
            }
        }
        return map;
    };
    const auto static_map = read_map();
    const auto local_map = read_map();
    std::size_t count = 0;
    double goal_yaw = 0.0;
    input >> count >> goal_yaw;
    std::vector<mini_nav_core::PathPoint> path(count);
    for (auto & point : path) input >> point.x >> point.y;
    auto map_pose = Pose(0.0, 0.0), odom_pose = Pose(0.0, 0.0), odom_from_map = Pose(0.0, 0.0);
    input >> map_pose.x >> map_pose.y >> map_pose.yaw;
    input >> odom_pose.x >> odom_pose.y >> odom_pose.yaw;
    input >> odom_from_map.x >> odom_from_map.y >> odom_from_map.yaw;
    double seconds = 0.0;
    input >> seconds;
    ASSERT_TRUE(input.good());
    ASSERT_FALSE(path.empty());
    mini_nav_core::PathTrackerParameters parameters;
    parameters.max_linear_speed = 0.15;
    parameters.max_angular_speed = 0.55;
    mini_nav_core::PathTracker tracker(parameters);
    tracker.SetPath(path, goal_yaw);
    const auto command = tracker.Step(map_pose, odom_pose, odom_from_map,
                                      static_map, local_map, seconds);
    EXPECT_EQ(command.status, mini_nav_core::TrackingStatus::kTracking);
    EXPECT_NEAR(command.linear_x, 0.0704701, 1e-6);
    EXPECT_NEAR(command.angular_z, 0.00221591, 1e-6);
}

/**
 * @brief 验证在目标位置容差内提前到达并停车。
 */
TEST(PathTracker, ReachesToleranceBeforeBlockedGoalCell)
{
    const auto static_map = ClearMap();
    auto local_map = ClearMap();
    local_map.SetCost(53, 50, 253);
    mini_nav_core::PathTracker tracker;
    tracker.SetPath({{0.0, 0.0}, {0.16, 0.0}}, 0.0);
    const auto approach = Step(tracker, static_map, local_map, Pose(0.0, 0.0), 0.0);
    EXPECT_EQ(approach.status, mini_nav_core::TrackingStatus::kTracking);
    EXPECT_GT(approach.linear_x, 0.0);
    const auto reached = Step(tracker, static_map, local_map, Pose(0.05, 0.0), 0.5);
    EXPECT_EQ(reached.status, mini_nav_core::TrackingStatus::kGoalReached);
    EXPECT_DOUBLE_EQ(reached.linear_x, 0.0);
}

/**
 * @brief 验证原地转向不被远处前视点错误阻挡。
 */
TEST(PathTracker, RotatesSafelyWhenDistantLookaheadIsBlocked)
{
    auto map = ClearMap();
    map.SetCost(53, 50, 254);
    mini_nav_core::PathTracker tracker;
    tracker.SetPath({{0.0, 0.0}, {1.0, 0.0}}, 0.0);
    const auto turn = Step(tracker, map, map, Pose(0.0, 0.0, 1.57), 0.0);
    EXPECT_EQ(turn.status, mini_nav_core::TrackingStatus::kTracking);
    EXPECT_DOUBLE_EQ(turn.linear_x, 0.0);
    EXPECT_LT(turn.angular_z, 0.0);
    map.SetCost(50, 50, 253);
    EXPECT_EQ(Step(tracker, map, map, Pose(0.0, 0.0, 1.57), 0.1).status,
              mini_nav_core::TrackingStatus::kCollisionRisk);
}

/**
 * @brief 验证预测实际候选圆弧而非到前视点的直线。
 */
TEST(PathTracker, ChecksCurvedCommandRatherThanStraightCarrotLine)
{
    mini_nav_core::Costmap2D map(200, 200, 0.005, -0.5, -0.5, 0);
    map.SetCost(116, 102, 254);
    mini_nav_core::PathTracker tracker;
    tracker.SetPath({{0.0, 0.0}, {std::cos(0.3), std::sin(0.3)}}, 0.3);
    const auto command = Step(tracker, map, map, Pose(0.0, 0.0), 0.0);
    EXPECT_EQ(command.status, mini_nav_core::TrackingStatus::kCollisionRisk);
    EXPECT_DOUBLE_EQ(command.linear_x, 0.0);
    EXPECT_DOUBLE_EQ(command.angular_z, 0.0);
}

/**
 * @brief 验证两次预测采样之间的连续扫掠不会漏障碍。
 */
TEST(PathTracker, ChecksWholeSegmentBetweenProjectionSamples)
{
    mini_nav_core::Costmap2D map(100, 100, 0.001, -0.05, -0.05, 0);
    map.SetCost(55, 50, 255);
    mini_nav_core::PathTrackerParameters parameters;
    parameters.max_linear_speed = 0.15;
    parameters.lookahead_distance = 0.03;
    mini_nav_core::PathTracker tracker(parameters);
    tracker.SetPath({{0.0, 0.0}, {1.0, 0.0}}, 0.0);
    const auto command = Step(tracker, map, map, Pose(0.0, 0.0), 0.0);
    EXPECT_EQ(command.status, mini_nav_core::TrackingStatus::kCollisionRisk);
    EXPECT_DOUBLE_EQ(command.linear_x, 0.0);
}

/**
 * @brief 验证短期预测不无故超出前视距离。
 */
TEST(PathTracker, LimitsProjectionToCarrotDistance)
{
    mini_nav_core::Costmap2D map(100, 100, 0.01, -0.5, -0.5, 0);
    map.SetCost(58, 50, 254);
    mini_nav_core::PathTrackerParameters parameters;
    parameters.lookahead_distance = 0.05;
    mini_nav_core::PathTracker tracker(parameters);
    tracker.SetPath({{0.0, 0.0}, {1.0, 0.0}}, 0.0);
    const auto command = Step(tracker, map, map, Pose(0.0, 0.0), 0.0);
    EXPECT_EQ(command.status, mini_nav_core::TrackingStatus::kTracking);
    EXPECT_GT(command.linear_x, 0.0);
}

/**
 * @brief 验证先限加速度，再检查实际候选轨迹与制动余量。
 */
TEST(PathTracker, LimitsAccelerationBeforeCheckingTheAppliedSweep)
{
    mini_nav_core::PathTrackerParameters parameters;
    parameters.max_linear_acceleration = 0.30;
    parameters.max_angular_acceleration = 1.0;
    mini_nav_core::PathTracker tracker(parameters);
    const auto map = ClearMap();
    tracker.SetPath({{0.0, 0.0}, {1.0, 0.0}}, 0.0);
    const auto first = Step(tracker, map, map, Pose(0.0, 0.0), 0.0);
    const auto second = Step(tracker, map, map, Pose(0.003, 0.0), 0.1);
    EXPECT_NEAR(first.linear_x, 0.03, 1e-9);
    EXPECT_NEAR(second.linear_x, 0.06, 1e-9);
    auto blocked = map;
    blocked.SetCost(50, 50, 253);
    const auto stop = Step(tracker, map, blocked, Pose(0.006, 0.0), 0.2);
    EXPECT_EQ(stop.status, mini_nav_core::TrackingStatus::kCollisionRisk);
    EXPECT_DOUBLE_EQ(stop.linear_x, 0.0); // Emergency stop bypasses deceleration smoothing.
    const auto restart = Step(tracker, map, map, Pose(0.006, 0.0), 0.3);
    EXPECT_NEAR(restart.linear_x, 0.03, 1e-9);
}
