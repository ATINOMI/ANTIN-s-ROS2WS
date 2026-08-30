#pragma once

#include <cstddef>

#include "mini_nav_core/localization/laser_model.hpp"

namespace mini_nav_core::localization
{

class LikelihoodFieldModel final : public LaserModel
{
public:
  LikelihoodFieldModel(
    double z_hit,
    double z_rand,
    double sigma_hit,
    std::size_t max_beams);

  void UpdateWeights(
    std::vector<Particle> & particles,
    const LaserScanData & scan,
    const LocalizationMap & map,
    const Pose2D & laser_pose_in_base) const override;

private:
  double z_hit_;
  double z_rand_;
  double sigma_hit_;
  std::size_t max_beams_;
};

}  // namespace mini_nav_core::localization
