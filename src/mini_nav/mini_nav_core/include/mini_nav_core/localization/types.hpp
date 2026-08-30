#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace mini_nav_core::localization
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

class Covariance3
{
public:
  Covariance3() = default;

  static Covariance3 Diagonal(double x, double y, double yaw)
  {
    Covariance3 covariance;
    covariance.values_[0] = x;
    covariance.values_[4] = y;
    covariance.values_[8] = yaw;
    return covariance;
  }

  double & At(std::size_t row, std::size_t column)
  {
    if (row >= 3U || column >= 3U) {
      throw std::out_of_range("Covariance index is out of bounds");
    }
    return values_[row * 3U + column];
  }

  double At(std::size_t row, std::size_t column) const
  {
    if (row >= 3U || column >= 3U) {
      throw std::out_of_range("Covariance index is out of bounds");
    }
    return values_[row * 3U + column];
  }

  const std::array<double, 9> & Values() const { return values_; }

private:
  std::array<double, 9> values_{};
};

struct Particle
{
  Pose2D pose;
  double weight{0.0};
};

struct PoseEstimate
{
  bool valid{false};
  Pose2D pose;
  Covariance3 covariance;
  double weight{0.0};
};

inline double NormalizeAngle(double angle)
{
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kTwoPi = 2.0 * kPi;
  angle = std::fmod(angle + kPi, kTwoPi);
  if (angle < 0.0) {
    angle += kTwoPi;
  }
  return angle - kPi;
}

inline double AngularDistance(double from, double to)
{
  return NormalizeAngle(from - to);
}

}  // namespace mini_nav_core::localization
