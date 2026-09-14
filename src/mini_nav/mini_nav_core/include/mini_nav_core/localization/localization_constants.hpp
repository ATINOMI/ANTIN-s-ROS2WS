#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>

namespace mini_nav_core::localization
{

// Shared numerical conventions for the ROS-independent localization core.
inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kTwoPi = 2.0 * kPi;
inline constexpr double kSqrtTwo = 1.41421356237309504880;
inline constexpr double kMinimumProbability = 1.0e-12;
inline constexpr double kMinimumPositiveDouble = std::numeric_limits<double>::min();
inline constexpr double kExpLowerLimit = -745.0;
inline constexpr double kExpUpperLimit = 709.0;
inline constexpr double kRayStepFraction = 0.5;
inline constexpr double kMinimumRayStepM = 1.0e-4;
inline constexpr double kMinimumTranslationForRotationM = 0.01;
inline constexpr double kDefaultTranslationBinSizeM = 0.5;
inline constexpr double kDefaultYawBinSizeRad = 15.0 * kPi / 180.0;
inline constexpr double kDefaultMaxObstacleDistanceM = 2.0;
inline constexpr double kDefaultMotionNoiseCoefficient = 0.2;
inline constexpr std::uint64_t kDefaultRandomSeed = 1U;
inline constexpr std::size_t kDefaultMinParticles = 500U;
inline constexpr std::size_t kDefaultMaxParticles = 2000U;
inline constexpr double kDefaultKldError = 0.05;
inline constexpr double kDefaultKldNormalQuantile = 2.33;
inline constexpr std::size_t kHashCombineConstant = 0x9e3779b9U;
inline constexpr double kCovarianceSymmetryTolerance = 1.0e-9;
inline constexpr double kCovariancePositiveSemidefiniteTolerance = 1.0e-10;


}  // namespace mini_nav_core::localization
