#pragma once

#include <vector>

#include "mini_nav_core/localization/types.hpp"

namespace mini_nav_core::localization
{

class MotionModel
{
public:
  virtual ~MotionModel() = default;

  virtual void UpdateParticles(
    std::vector<Particle> & particles,
    const Pose2D & previous_odom_pose,
    const Pose2D & current_odom_pose) = 0;
};

}  // namespace mini_nav_core::localization
