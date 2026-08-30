#pragma once

#include <cstdint>
#include <random>

#include "mini_nav_core/localization/motion_model.hpp"

namespace mini_nav_core::localization
{

class DifferentialMotionModel final : public MotionModel
{
public:
  explicit DifferentialMotionModel(
    double alpha1 = 0.2,
    double alpha2 = 0.2,
    double alpha3 = 0.2,
    double alpha4 = 0.2,
    double alpha5 = 0.2,
    std::uint64_t seed = 1);

  void UpdateParticles(
    std::vector<Particle> & particles,
    const Pose2D & previous_odom_pose,
    const Pose2D & current_odom_pose) override;

private:
  double alpha1_;
  double alpha2_;
  double alpha3_;
  double alpha4_;
  double alpha5_;
  std::mt19937_64 generator_;
};

}  // namespace mini_nav_core::localization
