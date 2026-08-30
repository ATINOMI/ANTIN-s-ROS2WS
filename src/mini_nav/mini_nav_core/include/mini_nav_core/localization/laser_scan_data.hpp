#pragma once

#include <vector>

namespace mini_nav_core::localization
{

struct LaserScanData
{
  std::vector<double> ranges;
  double angle_min{0.0};
  double angle_increment{0.0};
  double range_min{0.0};
  double range_max{0.0};
};

}  // namespace mini_nav_core::localization
