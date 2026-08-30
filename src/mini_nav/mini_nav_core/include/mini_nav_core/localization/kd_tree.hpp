#pragma once

#include <cstddef>
#include <unordered_map>
#include <vector>

#include "mini_nav_core/localization/types.hpp"

namespace mini_nav_core::localization
{

class KdTree
{
public:
  KdTree(double linear_bin_size = 0.5, double angular_bin_size = 0.261799387799);

  void Build(const std::vector<Particle> & particles);
  std::size_t GetOccupiedBinCount() const;

private:
  struct Key
  {
    int x;
    int y;
    int yaw;

    bool operator==(const Key & other) const
    {
      return x == other.x && y == other.y && yaw == other.yaw;
    }
  };

  struct KeyHash
  {
    std::size_t operator()(const Key & key) const;
  };

  Key MakeKey(const Pose2D & pose) const;

  double linear_bin_size_;
  double angular_bin_size_;
  std::unordered_map<Key, std::size_t, KeyHash> bin_ids_;
  std::vector<std::size_t> particle_bins_;
  std::vector<std::vector<std::size_t>> bin_particles_;
};

}  // namespace mini_nav_core::localization
