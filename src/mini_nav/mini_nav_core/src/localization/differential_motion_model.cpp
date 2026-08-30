#include "mini_nav_core/localization/differential_motion_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mini_nav_core::localization
{

namespace
{
void ValidateAlpha(double value, const char * name)
{
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(std::string(name) + " must be finite and non-negative");
  }
}

double SampleGaussian(std::mt19937_64 & generator, double variance)
{
  if (variance <= 0.0) {
    return 0.0;
  }
  std::normal_distribution<double> distribution(0.0, std::sqrt(variance));
  return distribution(generator);
}
}  // namespace

DifferentialMotionModel::DifferentialMotionModel(
  double alpha1, double alpha2, double alpha3, double alpha4, double alpha5,
  std::uint64_t seed)
: alpha1_(alpha1), alpha2_(alpha2), alpha3_(alpha3), alpha4_(alpha4), alpha5_(alpha5),
  generator_(seed)
{
  ValidateAlpha(alpha1_, "alpha1");
  ValidateAlpha(alpha2_, "alpha2");
  ValidateAlpha(alpha3_, "alpha3");
  ValidateAlpha(alpha4_, "alpha4");
  ValidateAlpha(alpha5_, "alpha5");
}

void DifferentialMotionModel::UpdateParticles(
  std::vector<Particle> & particles,
  const Pose2D & previous_odom_pose,
  const Pose2D & current_odom_pose)
{
  const double delta_x = current_odom_pose.x - previous_odom_pose.x;
  const double delta_y = current_odom_pose.y - previous_odom_pose.y;
  const double delta_trans = std::hypot(delta_x, delta_y);
  const double delta_rot1 = delta_trans < 0.01 ? 0.0 :
    AngularDistance(std::atan2(delta_y, delta_x), previous_odom_pose.yaw);
  const double delta_rot2 = AngularDistance(
    AngularDistance(current_odom_pose.yaw, previous_odom_pose.yaw), delta_rot1);

  const double rot1_noise = std::min(std::abs(AngularDistance(delta_rot1, 0.0)),
                                     std::abs(AngularDistance(delta_rot1, 3.14159265358979323846)));
  const double rot2_noise = std::min(std::abs(AngularDistance(delta_rot2, 0.0)),
                                     std::abs(AngularDistance(delta_rot2, 3.14159265358979323846)));

  const double rot1_variance = alpha1_ * rot1_noise * rot1_noise + alpha2_ * delta_trans * delta_trans;
  const double trans_variance = alpha3_ * delta_trans * delta_trans +
    alpha4_ * rot1_noise * rot1_noise + alpha4_ * rot2_noise * rot2_noise;
  const double rot2_variance = alpha1_ * rot2_noise * rot2_noise + alpha2_ * delta_trans * delta_trans;

  for (auto & particle : particles) {
    const double noisy_rot1 = AngularDistance(
      delta_rot1, SampleGaussian(generator_, rot1_variance));
    const double noisy_trans = delta_trans - SampleGaussian(generator_, trans_variance);
    const double noisy_rot2 = AngularDistance(
      delta_rot2, SampleGaussian(generator_, rot2_variance));

    particle.pose.x += noisy_trans * std::cos(particle.pose.yaw + noisy_rot1);
    particle.pose.y += noisy_trans * std::sin(particle.pose.yaw + noisy_rot1);
    particle.pose.yaw = NormalizeAngle(particle.pose.yaw + noisy_rot1 + noisy_rot2);
  }
}

}  // namespace mini_nav_core::localization
