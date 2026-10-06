/**
 * @file test_localization_naming.cpp
 * @brief 验证 localization_naming 模块行为及边界的测试。
 * @author Antinomy
 * @date 2026-10-01
 */
#include <algorithm>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "mini_nav_core/localization/amcl/beam_model.hpp"
#include "mini_nav_core/localization/amcl/kd_tree.hpp"
#include "mini_nav_core/localization/amcl/localization_constants.hpp"
#include "mini_nav_core/localization/amcl/localization_map.hpp"

using mini_nav_core::Costmap2D;
using mini_nav_core::localization::GridCell;
using mini_nav_core::localization::LocalizationMap;
using mini_nav_core::localization::Particle;
using mini_nav_core::localization::Pose2D;
using mini_nav_core::localization::PoseBinIndex;

/**
 * @brief 验证位姿分箱在负坐标及边界处的归组。
 */
TEST(PoseBinIndex, GroupsPoseCoordinatesAtSemanticBoundaries)
{
  PoseBinIndex index(0.5, mini_nav_core::localization::kPi / 2.0);
  const std::vector<Particle> particles{
    {Pose2D{-0.01, 0.0, 0.0}, 1.0},
    {Pose2D{-0.49, 0.0, 0.0}, 1.0},
    {Pose2D{0.50, 0.0, 0.0}, 1.0},
    {Pose2D{0.50, 0.0, mini_nav_core::localization::kPi}, 1.0}};

  index.BuildPoseBinIndex(particles);

  EXPECT_EQ(index.GetOccupiedBinCount(), 3U);
  index.Build(particles);
  EXPECT_EQ(index.GetOccupiedBinCount(), 3U);
}

/**
 * @brief 验证定位图空闲、占据、未知与图外距离查询。
 */
TEST(LocalizationMap, ExposesThreeStateCellsAndOutOfMapDistance)
{
  Costmap2D costmap(3, 3, 1.0, 0.0, 0.0, 0);
  costmap.SetCost(1, 1, LocalizationMap::kLethalObstacle);
  costmap.SetCost(2, 2, LocalizationMap::kUnknownCost);
  LocalizationMap map(costmap, 5.0);

  EXPECT_TRUE(map.IsKnownFree(GridCell{0, 0}));
  EXPECT_TRUE(map.IsKnownOccupied(GridCell{1, 1}));
  EXPECT_TRUE(map.IsUnknown(GridCell{2, 2}));
  EXPECT_FALSE(map.IsFree(GridCell{2, 2}));
  EXPECT_FALSE(map.IsOccupied(GridCell{2, 2}));

  double distance = -1.0;
  EXPECT_TRUE(map.TryGetObstacleDistanceAtWorld(0.5, 0.5, distance));
  EXPECT_GT(distance, 0.0);
  EXPECT_FALSE(map.TryGetObstacleDistanceAtWorld(-0.1, 0.5, distance));
  EXPECT_DOUBLE_EQ(map.GetObstacleDistanceAtWorld(-0.1, 0.5), 5.0);
}

/**
 * @brief 验证非有限位姿不能进入分箱索引。
 */
TEST(PoseBinIndex, RejectsNonFinitePoseCoordinates)
{
  PoseBinIndex index;
  const std::vector<Particle> particles{{
    Pose2D{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}, 1.0}};

  EXPECT_THROW(index.BuildPoseBinIndex(particles), std::invalid_argument);
}

/**
 * @brief 验证 Beam 模型拒绝全零及负混合权重。
 */
TEST(BeamModel, RejectsInvalidMixtureWeights)
{
  EXPECT_THROW(
    mini_nav_core::localization::BeamModel(0.0, 0.0, 0.0, 0.0, 0.2, 0.1, 10U),
    std::invalid_argument);
  EXPECT_THROW(
    mini_nav_core::localization::BeamModel(-0.1, 0.2, 0.2, 0.6, 0.2, 0.1, 10U),
    std::invalid_argument);
}

/**
 * @brief 验证零权重分箱不桥接两个定位假设。
 */
TEST(PoseBinIndex, DominantHypothesisDoesNotBridgeDisconnectedModes)
{
    PoseBinIndex bins;
    const std::vector<Particle> particles{
        {{0.0, 0.0, 0.0}, 0.4}, {{0.5, 0.0, 0.0}, 0.35},
        {{1.0, 0.0, 0.0}, 0.0}, {{1.5, 0.0, 0.0}, 0.0},
        {{2.0, 0.0, 0.0}, 0.25}};
    bins.Build(particles);
    const auto mode = bins.GetDominantParticleIndices(particles);
    ASSERT_EQ(mode.size(), 2u);
    EXPECT_NE(std::find(mode.begin(), mode.end(), 0u), mode.end());
    EXPECT_NE(std::find(mode.begin(), mode.end(), 1u), mode.end());
}

/**
 * @brief 验证偏航 ±π 两侧主簇按周期邻接。
 */
TEST(PoseBinIndex, ConnectsYawAcrossMinusPiAndPi)
{
    PoseBinIndex bins;
    const std::vector<Particle> particles{
        {{0.0, 0.0, 3.13}, 0.4}, {{0.0, 0.0, -3.13}, 0.4}, {{0.0, 0.0, 0.0}, 0.2}};
    bins.Build(particles);
    const auto mode = bins.GetDominantParticleIndices(particles);
    ASSERT_EQ(mode.size(), 2u);
    EXPECT_NE(std::find(mode.begin(), mode.end(), 0u), mode.end());
    EXPECT_NE(std::find(mode.begin(), mode.end(), 1u), mode.end());
}
