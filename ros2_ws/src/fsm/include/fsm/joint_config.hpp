// 机器人关节/驱动轮命名与索引的单一来源（SSOT）。
//
// 三套索引不要混用：
//   - steering_index 0..7  → SteeringCommand.joint_positions / joint_commands（8 转向关节）
//   - wheel_speed_index 0..3 → WheelSpeeds.speeds（4 驱动轮）
//   - webots_index 0..11   → hardware_bridge 在 Webots 中的电机遍历顺序
//
// 转向关节按数组顺序交替：节臂、轮臂、节臂、轮臂 …（奇数下标为轮臂）

#pragma once

#include <array>
#include <cstddef>

namespace fsm {

constexpr std::size_t kSteeringJointCount = 8;
constexpr std::size_t kDriveWheelCount = 4;
constexpr std::size_t kWebotsActuatorCount = 12;
constexpr int kNoWheelSpeed = -1;

enum class ArmType {
  kSectionArm,
  kWheelArm,
};

struct SteeringJointSpec {
  const char * name;
  ArmType arm_type;
};

// joint_commands / SteeringCommand 数组下标 → 关节名（节臂/轮臂交替）
constexpr std::array<SteeringJointSpec, kSteeringJointCount> kSteeringJoints{{
  {"link_002_joint", ArmType::kSectionArm},
  {"link_003_joint", ArmType::kWheelArm},
  {"link_005_joint", ArmType::kSectionArm},
  {"link_006_joint", ArmType::kWheelArm},
  {"link_008_joint", ArmType::kSectionArm},
  {"link_009_joint", ArmType::kWheelArm},
  {"link_011_joint", ArmType::kSectionArm},
  {"link_012_joint", ArmType::kWheelArm},
}};

// 自转 map_spin 只驱动轮臂
constexpr std::array<std::size_t, 4> kWheelArmIndices{1, 3, 5, 7};

struct DriveWheelSpec {
  const char * motor_name;
};

// WheelSpeeds.speeds 下标 → 驱动轮电机名（0/2 右侧，1/3 左侧，用于差速）
constexpr std::array<DriveWheelSpec, kDriveWheelCount> kDriveWheels{{
  {"link_004_joint"},
  {"link_007_joint"},
  {"link_010_joint"},
  {"link_013_joint"},
}};

struct WebotsActuatorSpec {
  const char * motor_name;
  const char * sensor_name;
  bool is_drive_wheel;
  int wheel_speed_index;
};

// Webots 模型树顺序：节臂 → 轮臂 → 驱动轮（重复 4 组）
constexpr std::array<WebotsActuatorSpec, kWebotsActuatorCount> kWebotsActuators{{
  {"link_002_joint", "link_002_joint_sensor", false, kNoWheelSpeed},
  {"link_003_joint", "link_003_joint_sensor", false, kNoWheelSpeed},
  {"link_004_joint", nullptr, true, 0},
  {"link_005_joint", "link_005_joint_sensor", false, kNoWheelSpeed},
  {"link_006_joint", "link_006_joint_sensor", false, kNoWheelSpeed},
  {"link_007_joint", nullptr, true, 1},
  {"link_008_joint", "link_008_joint_sensor", false, kNoWheelSpeed},
  {"link_009_joint", "link_009_joint_sensor", false, kNoWheelSpeed},
  {"link_010_joint", nullptr, true, 2},
  {"link_011_joint", "link_011_joint_sensor", false, kNoWheelSpeed},
  {"link_012_joint", "link_012_joint_sensor", false, kNoWheelSpeed},
  {"link_013_joint", nullptr, true, 3},
}};

inline bool IsWheelArm(std::size_t steering_index)
{
  return steering_index < kSteeringJoints.size() &&
         kSteeringJoints[steering_index].arm_type == ArmType::kWheelArm;
}

inline bool IsRightDriveWheel(std::size_t wheel_speed_index)
{
  return wheel_speed_index % 2 == 0;
}

inline const char * SteeringJointName(std::size_t steering_index)
{
  return steering_index < kSteeringJoints.size() ? kSteeringJoints[steering_index].name : "";
}

}  // namespace fsm
