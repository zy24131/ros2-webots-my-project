#ifndef MY_ROBOT_MAPS__SPIN_TO_JOINTS_HPP_
#define MY_ROBOT_MAPS__SPIN_TO_JOINTS_HPP_

#include "my_robot_maps/types.hpp"

namespace my_robot_maps {

// 自转：节臂保持 0，仅 4 个轮臂按对角线交替偏转
// steering_angle_deg 为幅度 (°)，|角|<3° 时全 0
JointAngles map_spin(double steering_angle_deg, SpinDirection direction);

}  // namespace my_robot_maps

#endif  // MY_ROBOT_MAPS__SPIN_TO_JOINTS_HPP_
