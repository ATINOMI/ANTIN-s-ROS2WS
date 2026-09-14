#include "mini_nav_core/localization/localization_map.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>

namespace mini_nav_core::localization
{

namespace
{
struct DistanceEntry
{
  double distance;
  std::size_t index;

  bool operator>(const DistanceEntry & other) const
  {
    return distance > other.distance;
  }
};
}  // namespace

LocalizationMap::LocalizationMap(const Costmap2D & costmap, double max_obstacle_distance)
: size_x_(costmap.GetSizeInCellsX()),
  size_y_(costmap.GetSizeInCellsY()),
  resolution_(costmap.GetResolution()),
  origin_x_(costmap.GetOriginX()),
  origin_y_(costmap.GetOriginY()),
  max_obstacle_distance_(max_obstacle_distance),
  costs_(costmap.GetCellCount(), 0),
  obstacle_distances_(costmap.GetCellCount(), max_obstacle_distance)
{
  if (!std::isfinite(max_obstacle_distance) || max_obstacle_distance <= 0.0) {
    throw std::invalid_argument("max_obstacle_distance must be finite and greater than zero");
  }

  for (unsigned int y = 0; y < size_y_; ++y) {
    for (unsigned int x = 0; x < size_x_; ++x) {
      const std::size_t index = static_cast<std::size_t>(y) * size_x_ + x;
      costs_[index] = costmap.GetCost(x, y);
      if (IsKnownFree(GridCell{x, y})) {
        free_cells_.push_back(GridCell{x, y});
      }
    }
  }

  std::priority_queue<DistanceEntry, std::vector<DistanceEntry>, std::greater<DistanceEntry>> queue;
  for (unsigned int y = 0; y < size_y_; ++y) {
    for (unsigned int x = 0; x < size_x_; ++x) {
      const GridCell cell{x, y};
      if (!IsKnownOccupied(cell)) {
        continue;
      }
      const std::size_t index = GetIndex(cell);
      obstacle_distances_[index] = 0.0;
      queue.push(DistanceEntry{0.0, index});
    }
  }

  struct NeighborOffset
  {
    int dx;
    int dy;
    double distance_multiplier;
  };
  constexpr std::array<NeighborOffset, 8> kNeighborOffsets{{
    {1, 0, 1.0}, {-1, 0, 1.0}, {0, 1, 1.0}, {0, -1, 1.0},
    {1, 1, kSqrtTwo}, {1, -1, kSqrtTwo},
    {-1, 1, kSqrtTwo}, {-1, -1, kSqrtTwo}}};

  while (!queue.empty()) {
    const auto current = queue.top();
    queue.pop();
    if (current.distance > obstacle_distances_[current.index]) {
      continue;
    }

    const unsigned int current_x = static_cast<unsigned int>(current.index % size_x_);
    const unsigned int current_y = static_cast<unsigned int>(current.index / size_x_);
    for (const auto & direction : kNeighborOffsets) {
      const int next_x = static_cast<int>(current_x) + direction.dx;
      const int next_y = static_cast<int>(current_y) + direction.dy;
      if (next_x < 0 || next_y < 0 || next_x >= static_cast<int>(size_x_) ||
          next_y >= static_cast<int>(size_y_)) {
        continue;
      }

      const double step = resolution_ * direction.distance_multiplier;
      const std::size_t next_index = static_cast<std::size_t>(next_y) * size_x_ + next_x;
      const double next_distance = std::min(max_obstacle_distance_, current.distance + step);
      if (next_distance < obstacle_distances_[next_index]) {
        obstacle_distances_[next_index] = next_distance;
        queue.push(DistanceEntry{next_distance, next_index});
      }
    }
  }
}

bool LocalizationMap::TryGetCellFromWorld(double world_x, double world_y, GridCell & cell) const
{
  if (!std::isfinite(world_x) || !std::isfinite(world_y) ||
      world_x < origin_x_ || world_y < origin_y_) {
    return false;
  }

  const double cell_x = (world_x - origin_x_) / resolution_;
  const double cell_y = (world_y - origin_y_) / resolution_;
  if (!std::isfinite(cell_x) || !std::isfinite(cell_y) ||
      cell_x < 0.0 || cell_y < 0.0 ||
      cell_x >= static_cast<double>(size_x_) || cell_y >= static_cast<double>(size_y_)) {
    return false;
  }

  cell = GridCell{
    static_cast<unsigned int>(cell_x),
    static_cast<unsigned int>(cell_y)};
  return true;
}

void LocalizationMap::GetCellCenter(const GridCell & cell, double & world_x, double & world_y) const
{
  if (!IsInBounds(cell)) {
    throw std::out_of_range("Localization map cell is out of bounds");
  }
  world_x = origin_x_ + (static_cast<double>(cell.x) + 0.5) * resolution_;
  world_y = origin_y_ + (static_cast<double>(cell.y) + 0.5) * resolution_;
}

bool LocalizationMap::IsKnownFree(const GridCell & cell) const
{
  // Unknown cost is above the lethal threshold, so this test means known free.
  return IsInBounds(cell) && costs_[GetIndex(cell)] < kLethalObstacle;
}

bool LocalizationMap::IsKnownOccupied(const GridCell & cell) const
{
  return IsInBounds(cell) && costs_[GetIndex(cell)] >= kLethalObstacle &&
         costs_[GetIndex(cell)] != kUnknownCost;
}

bool LocalizationMap::IsUnknown(const GridCell & cell) const
{
  return IsInBounds(cell) && costs_[GetIndex(cell)] == kUnknownCost;
}

bool LocalizationMap::TryGetObstacleDistanceAtWorld(
  double world_x, double world_y, double & distance_m) const
{
  GridCell cell;
  if (!TryGetCellFromWorld(world_x, world_y, cell)) {
    return false;
  }
  distance_m = obstacle_distances_[GetIndex(cell)];
  return true;
}

double LocalizationMap::GetObstacleDistanceAtWorld(double world_x, double world_y) const
{
  double distance_m = max_obstacle_distance_;
  TryGetObstacleDistanceAtWorld(world_x, world_y, distance_m);
  return distance_m;
}

bool LocalizationMap::CastRay(
  const Pose2D & ray_pose, double max_range_m, double & expected_range_m) const
{
  if (!std::isfinite(max_range_m) || max_range_m <= 0.0) {
    return false;
  }

  const double step = std::max(resolution_ * kRayStepFraction, kMinimumRayStepM);
  for (double distance = 0.0; distance <= max_range_m; distance += step) {
    const double x = ray_pose.x + distance * std::cos(ray_pose.yaw);
    const double y = ray_pose.y + distance * std::sin(ray_pose.yaw);
    GridCell cell;
    if (!TryGetCellFromWorld(x, y, cell)) {
      expected_range_m = distance;
      return true;
    }
    if (IsKnownOccupied(cell)) {
      expected_range_m = distance;
      return true;
    }
  }

  expected_range_m = max_range_m;
  return true;
}

bool LocalizationMap::SampleKnownFreeCell(std::mt19937_64 & generator, GridCell & cell) const
{
  if (free_cells_.empty()) {
    return false;
  }
  std::uniform_int_distribution<std::size_t> distribution(0, free_cells_.size() - 1U);
  cell = free_cells_[distribution(generator)];
  return true;
}

std::size_t LocalizationMap::GetKnownFreeCellCount() const
{
  return free_cells_.size();
}

unsigned int LocalizationMap::GetSizeInCellsX() const
{
  return size_x_;
}

unsigned int LocalizationMap::GetSizeInCellsY() const
{
  return size_y_;
}

double LocalizationMap::GetResolution() const
{
  return resolution_;
}

double LocalizationMap::GetMaxObstacleDistance() const
{
  return max_obstacle_distance_;
}

std::size_t LocalizationMap::GetIndex(const GridCell & cell) const
{
  return static_cast<std::size_t>(cell.y) * size_x_ + cell.x;
}

bool LocalizationMap::IsInBounds(const GridCell & cell) const
{
  return cell.x < size_x_ && cell.y < size_y_;
}

}  // namespace mini_nav_core::localization
