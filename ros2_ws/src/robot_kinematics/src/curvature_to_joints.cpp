#include <cmath>

#include "robot_kinematics/curvature_to_joints.hpp"
#include "robot_kinematics/narrow_track_maps.hpp"
#include "robot_kinematics/wide_track_maps.hpp"

namespace robot_kinematics {

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

    for (size_t i = 0; i < angles.size(); i += 2) {
      angles[i] = signed_deg_to_rad(arm_deg, sign);
      angles[i + 1] = signed_deg_to_rad(wheel_deg, sign);
    }
  } else {
    const double wheel_deg = narrow::curvature_to_wheel_out(k);
    const double middle = narrow::wheel_out_to_middle(wheel_deg);
    const double arm_deg = narrow::middle_to_arm_in(middle);

    for (size_t i = 0; i < angles.size(); i += 2) {
      angles[i] = signed_deg_to_rad(arm_deg, sign);
      angles[i + 1] = signed_deg_to_rad(wheel_deg, sign);
    }
  }

  return angles;
}

}  // namespace robot_kinematics
