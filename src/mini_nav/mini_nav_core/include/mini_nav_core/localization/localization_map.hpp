#pragma once

#include <cstddef>
#include <random>
#include <vector>

#include "mini_nav_core/localization/types.hpp"
#include "mini_nav_core/map/costmap_2d.hpp"

namespace mini_nav_core::localization
{

struct MapCell
{
  unsigned int x{0};
  unsigned int y{0};
};

class LocalizationMap
{
public:
  static constexpr unsigned char kLethalObstacle = 254;
  static constexpr unsigned char kUnknownCost = 255;

  explicit LocalizationMap(
    const Costmap2D & costmap,
    double max_obstacle_distance = 2.0);

  bool TryGetCellFromWorld(double world_x, double world_y, MapCell & cell) const;
  void GetCellCenter(const MapCell & cell, double & world_x, double & world_y) const;
  bool IsFree(const MapCell & cell) const;
  bool IsOccupied(const MapCell & cell) const;
  double GetObstacleDistanceAtWorld(double world_x, double world_y) const;
  bool CastRay(const Pose2D & pose, double max_range, double & range) const;

  bool SampleFreeCell(std::mt19937_64 & generator, MapCell & cell) const;
  std::size_t GetFreeCellCount() const;
  unsigned int GetSizeInCellsX() const;
  unsigned int GetSizeInCellsY() const;
  double GetResolution() const;
  double GetMaxObstacleDistance() const;

private:
  std::size_t GetIndex(const MapCell & cell) const;
  bool IsInBounds(const MapCell & cell) const;

  unsigned int size_x_{0};
  unsigned int size_y_{0};
  double resolution_{0.0};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double max_obstacle_distance_{0.0};
  std::vector<unsigned char> costs_;
  std::vector<double> obstacle_distances_;
  std::vector<MapCell> free_cells_;
};

}  // namespace mini_nav_core::localization
