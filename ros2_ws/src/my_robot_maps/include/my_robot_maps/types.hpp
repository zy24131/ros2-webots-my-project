#ifndef MY_ROBOT_MAPS__TYPES_HPP_
#define MY_ROBOT_MAPS__TYPES_HPP_

#include <array>

namespace my_robot_maps {

// 8 个转向关节角 (rad)，顺序对应 1~8 号位置（见 README 转向关节标定）
using JointAngles = std::array<double, 8>;

enum class TrackMode {
  kNarrow,
  kWide,
};

enum class SpinDirection {
  kLeft,
  kRight,
};

}  // namespace my_robot_maps

#endif  // MY_ROBOT_MAPS__TYPES_HPP_
