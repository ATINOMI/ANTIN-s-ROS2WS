#pragma once

#include <cstddef>
#include <random>
#include <vector>

#include "mini_nav_core/localization/localization_constants.hpp"
#include "mini_nav_core/localization/types.hpp"
#include "mini_nav_core/map/costmap_2d.hpp"

namespace mini_nav_core::localization
{

struct GridCell
{
  unsigned int x{0};
  unsigned int y{0};
};

// MapCell is retained as a source-compatible alias; fields are grid indices.
using MapCell = GridCell;

class LocalizationMap
{
public:
  static constexpr unsigned char kLethalObstacle = 254;
  static constexpr unsigned char kUnknownCost = 255;

  explicit LocalizationMap(
    const Costmap2D & costmap,
    double max_obstacle_distance = kDefaultMaxObstacleDistanceM);

  bool TryGetCellFromWorld(double world_x, double world_y, GridCell & cell) const;
  void GetCellCenter(const GridCell & cell, double & world_x, double & world_y) const;
  bool IsKnownFree(const GridCell & cell) const;
  bool IsKnownOccupied(const GridCell & cell) const;
  bool IsUnknown(const GridCell & cell) const;

  // Compatibility wrappers for the original map-state names.
  bool IsFree(const GridCell & cell) const { return IsKnownFree(cell); }
  bool IsOccupied(const GridCell & cell) const { return IsKnownOccupied(cell); }

  bool TryGetObstacleDistanceAtWorld(
    double world_x, double world_y, double & distance_m) const;
  // Returns max_obstacle_distance for an out-of-map query for compatibility.
  double GetObstacleDistanceAtWorld(double world_x, double world_y) const;
  bool CastRay(
    const Pose2D & ray_pose, double max_range_m, double & expected_range_m) const;

  bool SampleKnownFreeCell(std::mt19937_64 & generator, GridCell & cell) const;
  std::size_t GetKnownFreeCellCount() const;
  // Compatibility wrappers for the original free-cell names.
  bool SampleFreeCell(std::mt19937_64 & generator, GridCell & cell) const
  {
    return SampleKnownFreeCell(generator, cell);
  }
  std::size_t GetFreeCellCount() const { return GetKnownFreeCellCount(); }
  unsigned int GetSizeInCellsX() const;
  unsigned int GetSizeInCellsY() const;
  double GetResolution() const;
  double GetMaxObstacleDistance() const;

private:
  std::size_t GetIndex(const GridCell & cell) const;
  bool IsInBounds(const GridCell & cell) const;

  unsigned int size_x_{0};
  unsigned int size_y_{0};
  double resolution_{0.0};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double max_obstacle_distance_{0.0};
  std::vector<unsigned char> costs_;
  std::vector<double> obstacle_distances_;
  std::vector<GridCell> free_cells_;
};

}  // namespace mini_nav_core::localization
