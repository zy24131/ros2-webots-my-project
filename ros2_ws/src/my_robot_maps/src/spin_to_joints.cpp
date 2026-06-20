// 自转关节映射：节臂 0，仅轮臂 (2,4,6,8) 对角交替偏转

#include <algorithm>
#include <array>
#include <cmath>

#include "my_robot_maps/spin_to_joints.hpp"

namespace my_robot_maps {

namespace {

constexpr double kMaxAngle = 90.0;
constexpr double kDeadband = 3.0;
constexpr double kDegToRad = M_PI / 180.0;

// JointAngles 索引：1,3,5,7 对应位置 2,4,6,8 轮臂
constexpr std::array<int, 4> kWheelArmIndices = {1, 3, 5, 7};

double WheelMagnitudeRad(double steering_angle_deg)
{
  const double magnitude = std::clamp(std::abs(steering_angle_deg), 0.0, kMaxAngle);
  if (magnitude < kDeadband) {
    return 0.0;
  }
  return magnitude * kDegToRad;
}

}  // namespace

JointAngles map_spin(double steering_angle_deg, SpinDirection direction)
{
  JointAngles angles{};
  const double m = WheelMagnitudeRad(steering_angle_deg);
  if (m < 1e-12) {
    return angles;
  }

  // spin_left:  2,-  4,+  6,-  8,+
  // spin_right: 2,+  4,-  6,+  8,-
  const std::array<double, 4> signs =
    (direction == SpinDirection::kLeft) ?
    std::array<double, 4>{-1.0, 1.0, -1.0, 1.0} :
    std::array<double, 4>{1.0, -1.0, 1.0, -1.0};

  for (size_t i = 0; i < kWheelArmIndices.size(); ++i) {
    angles[kWheelArmIndices[i]] = signs[i] * m;
  }

  return angles;
}

}  // namespace my_robot_maps
