#ifndef MY_ROBOT_MAPS__CURVATURE_TO_JOINTS_HPP_
#define MY_ROBOT_MAPS__CURVATURE_TO_JOINTS_HPP_

#include "my_robot_maps/types.hpp"

namespace my_robot_maps {

// 曲率 κ + 轮距模式 → 8 关节角；函数名 map，占位待填真实几何
JointAngles map(double curvature, TrackMode track_mode);

}  // namespace my_robot_maps

#endif  // MY_ROBOT_MAPS__CURVATURE_TO_JOINTS_HPP_
