// 宽轮距映射链 — 源自 al.cpp (Project_xujunming/webots)

#include <algorithm>
#include <cmath>

#include "my_robot_maps/wide_track_maps.hpp"

namespace my_robot_maps {
namespace wide {

namespace {

constexpr double kMaxAngle = 90.0;
constexpr double kDeadband = 3.0;
constexpr double kCMax = 1.0 / 2.5;

}  // namespace

double steering_angle_to_curvature(double steering_angle_deg)
{
  const double angle = std::clamp(std::abs(steering_angle_deg), 0.0, kMaxAngle);

  if (angle < kDeadband) {
    return 0.0;
  }

  double curvature = kCMax * (angle / kMaxAngle);
  if (curvature > kCMax) {
    curvature = kCMax;
  }
  return curvature;
}

double curvature_to_arm_out(double curvature)
{
  const double x = curvature;
  return -25.06 * x * x - 35.16 * x + 34.79;
}

double arm_out_to_middle(double arm_out)
{
  const double x = arm_out;
  return 317.4 * std::exp(0.09752 * x) + 3.982e-09 * std::exp(0.8688 * x);
}

double middle_to_wheel_in(double middle)
{
  const double x = middle;
  if (x < 7000.0) {
    return 20.46 * std::exp(5.272e-05 * x) - 130.3 * std::exp(-0.001119 * x);
  }
  return 34.14 * std::exp(7.336e-08 * x) - 9.832 * std::exp(-0.0001103 * x);
}

}  // namespace wide
}  // namespace my_robot_maps
