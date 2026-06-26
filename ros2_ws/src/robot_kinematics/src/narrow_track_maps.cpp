// 窄轮距映射链 — 源自 al.cpp (Project_xujunming/webots)

#include <algorithm>
#include <cmath>

#include "robot_kinematics/narrow_track_maps.hpp"

namespace robot_kinematics {
namespace narrow {

namespace {

constexpr double kMaxAngle = 90.0;
constexpr double kDeadband = 3.0;
constexpr double kAngleClamp = 80.0;
constexpr double kCMax = 1.0 / 2.14;

}  // namespace

double steering_angle_to_curvature(double steering_angle_deg)
{
  double angle = std::abs(steering_angle_deg);
  if (angle > kAngleClamp) {
    angle = kAngleClamp;
  }
  angle = std::clamp(angle, 0.0, kMaxAngle);

  if (angle < kDeadband) {
    return 0.0;
  }

  double curvature = kCMax * (angle / kMaxAngle);
  if (curvature > kCMax) {
    curvature = kCMax;
  }
  return curvature;
}

double curvature_to_wheel_out(double curvature)
{
  const double x = curvature;
  return 29.17 * x * x + 31.18 * x + 1.354;
}

double wheel_out_to_middle(double wheel_out)
{
  const double x = wheel_out;
  return 19330.0 * std::exp(-0.3318 * x) + 5657.0 * std::exp(-0.06546 * x);
}

double middle_to_arm_in(double middle)
{
  const double x = middle;
  return 195.1 * std::exp(-0.001595 * x) + 27.96 * std::exp(-0.0002196 * x);
}

}  // namespace narrow
}  // namespace robot_kinematics
