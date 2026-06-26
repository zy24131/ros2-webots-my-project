#ifndef MY_ROBOT_MAPS__TYPES_HPP_
#define MY_ROBOT_MAPS__TYPES_HPP_

#include <array>

namespace robot_kinematics {

// 8 个转向关节角 (rad)，顺序见 fsm/joint_config.hpp → kSteeringJoints
using JointAngles = std::array<double, 8>;

enum class TrackMode {
  kNarrow,
  kWide,
};

enum class SpinDirection {
  kLeft,
  kRight,
};

}  // namespace robot_kinematics

#endif  // MY_ROBOT_MAPS__TYPES_HPP_
