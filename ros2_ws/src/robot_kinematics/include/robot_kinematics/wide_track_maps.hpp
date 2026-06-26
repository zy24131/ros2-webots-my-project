#ifndef MY_ROBOT_MAPS__WIDE_TRACK_MAPS_HPP_
#define MY_ROBOT_MAPS__WIDE_TRACK_MAPS_HPP_

namespace robot_kinematics {
namespace wide {

double steering_angle_to_curvature(double steering_angle_deg);

double curvature_to_arm_out(double curvature);
double arm_out_to_middle(double arm_out);
double middle_to_wheel_in(double middle);

}  // namespace wide
}  // namespace robot_kinematics

#endif  // MY_ROBOT_MAPS__WIDE_TRACK_MAPS_HPP_
