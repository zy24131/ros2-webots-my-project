#ifndef MY_ROBOT_MAPS__NARROW_TRACK_MAPS_HPP_
#define MY_ROBOT_MAPS__NARROW_TRACK_MAPS_HPP_

namespace robot_kinematics {
namespace narrow {

// 转向角 (°，取绝对值参与计算) → 曲率 κ (1/m)，符号由调用方处理
double steering_angle_to_curvature(double steering_angle_deg);

double curvature_to_wheel_out(double curvature);
double wheel_out_to_middle(double wheel_out);
double middle_to_arm_in(double middle);

}  // namespace narrow
}  // namespace robot_kinematics

#endif  // MY_ROBOT_MAPS__NARROW_TRACK_MAPS_HPP_
