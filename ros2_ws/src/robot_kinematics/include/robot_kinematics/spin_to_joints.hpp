#ifndef MY_ROBOT_MAPS__SPIN_TO_JOINTS_HPP_
#define MY_ROBOT_MAPS__SPIN_TO_JOINTS_HPP_

#include "robot_kinematics/types.hpp"

namespace robot_kinematics {

// 自转：节臂 0，仅 4 个轮臂对角 ±10°（不跟方向盘）
JointAngles map_spin(double steering_angle_deg, SpinDirection direction);

}  // namespace robot_kinematics

#endif  // MY_ROBOT_MAPS__SPIN_TO_JOINTS_HPP_
