#include "mini_nav_core/localization/beam_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace mini_nav_core::localization
{

namespace
{
Pose2D ComposePose(const Pose2D & base, const Pose2D & relative)
{
  const double cosine = std::cos(base.yaw);
  const double sine = std::sin(base.yaw);
  return Pose2D{
    base.x + cosine * relative.x - sine * relative.y,
    base.y + sine * relative.x + cosine * relative.y,
    NormalizeAngle(base.yaw + relative.yaw)};
}

double Gaussian(double difference, double sigma)
{
  return std::exp(-0.5 * difference * difference / (sigma * sigma)) /
         (std::sqrt(2.0 * 3.14159265358979323846) * sigma);
}
}  // namespace

BeamModel::BeamModel(
  double z_hit, double z_short, double z_max, double z_rand,
  double sigma_hit, double lambda_short, std::size_t max_beams)
: z_hit_(z_hit), z_short_(z_short), z_max_(z_max), z_rand_(z_rand),
  sigma_hit_(sigma_hit), lambda_short_(lambda_short), max_beams_(max_beams)
{
  if (!std::isfinite(sigma_hit_) || sigma_hit_ <= 0.0 ||
      !std::isfinite(lambda_short_) || lambda_short_ <= 0.0 || max_beams_ == 0U) {
    throw std::invalid_argument("Invalid beam model parameters");
  }
}

void BeamModel::UpdateWeights(
  std::vector<Particle> & particles,
  const LaserScanData & scan,
  const LocalizationMap & map,
  const Pose2D & laser_pose_in_base) const
{
  if (particles.empty() || scan.ranges.empty() || scan.range_max <= 0.0 ||
      !std::isfinite(scan.range_max)) {
    return;
  }

  const std::size_t step = std::max<std::size_t>(
    1U, (scan.ranges.size() - 1U) / std::max<std::size_t>(1U, max_beams_ - 1U));
  for (auto & particle : particles) {
    const Pose2D laser_pose = ComposePose(particle.pose, laser_pose_in_base);
    double log_weight = std::log(std::max(particle.weight, std::numeric_limits<double>::min()));
    for (std::size_t index = 0; index < scan.ranges.size(); index += step) {
      const double observed_range = scan.ranges[index];
      if (!std::isfinite(observed_range) || observed_range < scan.range_min) {
        continue;
      }

      const double bearing = scan.angle_min + static_cast<double>(index) * scan.angle_increment;
      const Pose2D beam_pose{
        laser_pose.x, laser_pose.y, NormalizeAngle(laser_pose.yaw + bearing)};
      double expected_range = scan.range_max;
      map.CastRay(beam_pose, scan.range_max, expected_range);

      double probability = z_hit_ * Gaussian(observed_range - expected_range, sigma_hit_);
      if (observed_range <= expected_range) {
        probability += z_short_ * lambda_short_ *
          std::exp(-lambda_short_ * observed_range);
      }
      if (observed_range >= scan.range_max) {
        probability += z_max_;
      }
      probability += z_rand_ / scan.range_max;
      log_weight += std::log(std::max(probability, 1.0e-12));
    }
    particle.weight = std::exp(std::max(-745.0, std::min(709.0, log_weight)));
  }
}

}  // namespace mini_nav_core::localization
