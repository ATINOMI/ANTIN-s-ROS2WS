#pragma once

#include <stdexcept>
#include <vector>

#include "mini_nav_core/localization/laser_scan_data.hpp"
#include "mini_nav_core/localization/localization_map.hpp"

namespace mini_nav_core::localization
{

class LaserModel
{
public:
  virtual ~LaserModel() = default;

  /**
   * @brief Multiply each particle weight by its scan measurement likelihood.
   *
   * The laser pose is the transform from the robot base frame to the laser
   * frame, expressed in metres and radians.
   */
  virtual void ApplyMeasurementLikelihood(
    std::vector<Particle> & particles,
    const LaserScanData & scan,
    const LocalizationMap & map,
    const Pose2D & base_to_laser_pose) const
  {
    // Dispatch to legacy models that still override UpdateWeights().
    UpdateWeights(particles, scan, map, base_to_laser_pose);
  }

  /**
   * @brief Compatibility entry point for the original method name.
   *
   * New models override ApplyMeasurementLikelihood(); legacy models may still
   * override this method and are reached through the canonical entry point.
   */
  virtual void UpdateWeights(
    std::vector<Particle> & particles,
    const LaserScanData & scan,
    const LocalizationMap & map,
    const Pose2D & base_to_laser_pose) const
  {
    (void)particles;
    (void)scan;
    (void)map;
    (void)base_to_laser_pose;
    throw std::logic_error(
      "LaserModel must override ApplyMeasurementLikelihood or UpdateWeights");
  }
};

}  // namespace mini_nav_core::localization
