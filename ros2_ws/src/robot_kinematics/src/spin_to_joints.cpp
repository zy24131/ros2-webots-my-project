// 自转关节映射：节臂 0，仅轮臂对角 ±10°（索引见 fsm::kWheelArmIndices）

#include <array>
#include <cmath>

#include "fsm/joint_config.hpp"
#include "robot_kinematics/spin_to_joints.hpp"

namespace robot_kinematics {

namespace {

constexpr double kSpinWheelArmDeg = 10.0;
constexpr double kDegToRad = M_PI / 180.0;

}  // namespace

JointAngles map_spin(double /*steering_angle_deg*/, SpinDirection direction)
{
  JointAngles angles{};
  const double m = kSpinWheelArmDeg * kDegToRad;

  // spin_left:  2,-  4,+  6,-  8,+
  // spin_right: 2,+  4,-  6,+  8,-
  const std::array<double, 4> signs =
    (direction == SpinDirection::kLeft) ?
    std::array<double, 4>{-1.0, 1.0, -1.0, 1.0} :
    std::array<double, 4>{1.0, -1.0, 1.0, -1.0};

  for (size_t i = 0; i < fsm::kWheelArmIndices.size(); ++i) {
    angles[fsm::kWheelArmIndices[i]] = signs[i] * m;
  }

  return angles;
}

}  // namespace robot_kinematics
