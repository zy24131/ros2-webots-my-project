#ifndef MY_ROBOT_MAPS__SPIN_TO_JOINTS_HPP_
#define MY_ROBOT_MAPS__SPIN_TO_JOINTS_HPP_

#include "my_robot_maps/types.hpp"

namespace my_robot_maps {

// 自转：节臂 0，仅 4 个轮臂对角 ±10°（不跟方向盘）
JointAngles map_spin(double steering_angle_deg, SpinDirection direction);

}  // namespace my_robot_maps

#endif  // MY_ROBOT_MAPS__SPIN_TO_JOINTS_HPP_
