#pragma once

#include <vector>

#include "mini_nav_core/localization/laser_scan_data.hpp"
#include "mini_nav_core/localization/localization_map.hpp"

namespace mini_nav_core::localization
{

class LaserModel
{
public:
  virtual ~LaserModel() = default;

  virtual void UpdateWeights(
    std::vector<Particle> & particles,
    const LaserScanData & scan,
    const LocalizationMap & map,
    const Pose2D & laser_pose_in_base) const = 0;
};

}  // namespace mini_nav_core::localization
