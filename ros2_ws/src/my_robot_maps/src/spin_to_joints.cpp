// 自转关节映射：节臂 0，仅轮臂 (2,4,6,8) 对角 ±10°
//
// joint_positions 下标 → Webots 关节（actuator_executor kJoints 顺序）:
//   [0] link_002 节臂   [1] link_003 轮臂 ← 自转有指令
//   [2] link_005 节臂   [3] link_006 轮臂 ← 自转有指令
//   [4] link_008 节臂   [5] link_009 轮臂 ← 自转有指令
//   [6] link_011 节臂   [7] link_012 轮臂 ← 自转有指令

#include <array>
#include <cmath>

#include "my_robot_maps/spin_to_joints.hpp"

namespace my_robot_maps {

namespace {

constexpr double kSpinWheelArmDeg = 10.0;
constexpr double kDegToRad = M_PI / 180.0;

// JointAngles 索引：1,3,5,7 对应位置 2,4,6,8 轮臂
constexpr std::array<int, 4> kWheelArmIndices = {1, 3, 5, 7};

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

  for (size_t i = 0; i < kWheelArmIndices.size(); ++i) {
    angles[kWheelArmIndices[i]] = signs[i] * m;
  }

  return angles;
}

}  // namespace my_robot_maps
