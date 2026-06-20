#ifndef MY_ROBOT_MAPS__STEERING_ANGLE_TO_CURVATURE_HPP_
#define MY_ROBOT_MAPS__STEERING_ANGLE_TO_CURVATURE_HPP_

#include "my_robot_maps/types.hpp"

namespace my_robot_maps {

// 有符号转向角 (°) + 轮距模式 → 有符号曲率 κ (1/m)
double steering_angle_to_curvature(double steering_angle_deg, TrackMode track_mode);

}  // namespace my_robot_maps

#endif  // MY_ROBOT_MAPS__STEERING_ANGLE_TO_CURVATURE_HPP_
