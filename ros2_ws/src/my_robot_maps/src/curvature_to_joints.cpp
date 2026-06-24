#include <cmath>

#include "my_robot_maps/curvature_to_joints.hpp"
#include "my_robot_maps/narrow_track_maps.hpp"
#include "my_robot_maps/wide_track_maps.hpp"

namespace my_robot_maps {

namespace {

constexpr double kDegToRad = M_PI / 180.0;

double signed_deg_to_rad(double deg, double sign)
{
  return sign * deg * kDegToRad;
}

}  // namespace

JointAngles map(double curvature, TrackMode track_mode)
{
  JointAngles angles{};

  if (std::abs(curvature) < 1e-12) {
    return angles;
  }

  const double sign = (curvature >= 0.0) ? 1.0 : -1.0;
  const double k = std::abs(curvature);

  if (track_mode == TrackMode::kWide) {
    const double arm_deg = wide::curvature_to_arm_out(k);
    const double middle = wide::arm_out_to_middle(arm_deg);
    const double wheel_deg = wide::middle_to_wheel_in(middle);

    // 1,3,5,7 节臂 ← Arm_Out；2,4,6,8 轮臂 ← Wheel_In
    angles[0] = signed_deg_to_rad(arm_deg, sign);
    angles[1] = signed_deg_to_rad(wheel_deg, sign);
    angles[2] = signed_deg_to_rad(arm_deg, sign);
    angles[3] = signed_deg_to_rad(wheel_deg, sign);
    angles[4] = signed_deg_to_rad(arm_deg, sign);
    angles[5] = signed_deg_to_rad(wheel_deg, sign);
    angles[6] = signed_deg_to_rad(arm_deg, sign);
    angles[7] = signed_deg_to_rad(wheel_deg, sign);
  } else {
    //窄轮距
    const double wheel_deg = narrow::curvature_to_wheel_out(k);
    const double middle = narrow::wheel_out_to_middle(wheel_deg);
    const double arm_deg = narrow::middle_to_arm_in(middle);

    // 1,3,5,7 节臂 ← Arm_In；2,4,6,8 轮臂 ← Wheel_Out
    angles[0] = signed_deg_to_rad(arm_deg, sign);
    angles[1] = signed_deg_to_rad(wheel_deg, sign);
    angles[2] = signed_deg_to_rad(arm_deg, sign);
    angles[3] = signed_deg_to_rad(wheel_deg, sign);
    angles[4] = signed_deg_to_rad(arm_deg, sign);
    angles[5] = signed_deg_to_rad(wheel_deg, sign);
    angles[6] = signed_deg_to_rad(arm_deg, sign);
    angles[7] = signed_deg_to_rad(wheel_deg, sign);
  }

  return angles;
}

}  // namespace my_robot_maps
