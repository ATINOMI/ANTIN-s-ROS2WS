#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "mini_nav_core/localization/beam_model.hpp"
#include "mini_nav_core/localization/kd_tree.hpp"
#include "mini_nav_core/localization/localization_constants.hpp"
#include "mini_nav_core/localization/localization_map.hpp"

using mini_nav_core::Costmap2D;
using mini_nav_core::localization::GridCell;
using mini_nav_core::localization::LocalizationMap;
using mini_nav_core::localization::Particle;
using mini_nav_core::localization::Pose2D;
using mini_nav_core::localization::PoseBinIndex;

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

TEST(PoseBinIndex, RejectsNonFinitePoseCoordinates)
{
  PoseBinIndex index;
  const std::vector<Particle> particles{{
    Pose2D{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}, 1.0}};

  EXPECT_THROW(index.BuildPoseBinIndex(particles), std::invalid_argument);
}

TEST(BeamModel, RejectsInvalidMixtureWeights)
{
  EXPECT_THROW(
    mini_nav_core::localization::BeamModel(0.0, 0.0, 0.0, 0.0, 0.2, 0.1, 10U),
    std::invalid_argument);
  EXPECT_THROW(
    mini_nav_core::localization::BeamModel(-0.1, 0.2, 0.2, 0.6, 0.2, 0.1, 10U),
    std::invalid_argument);
}
