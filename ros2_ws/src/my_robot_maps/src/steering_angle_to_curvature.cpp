#include <cmath>

#include "my_robot_maps/narrow_track_maps.hpp"
#include "my_robot_maps/steering_angle_to_curvature.hpp"
#include "my_robot_maps/wide_track_maps.hpp"

namespace my_robot_maps {

double steering_angle_to_curvature(double steering_angle_deg, TrackMode track_mode)
{
  if (std::abs(steering_angle_deg) < 1e-9) {
    return 0.0;
  }

  const double sign = (steering_angle_deg >= 0.0) ? 1.0 : -1.0;
  double magnitude = 0.0;

  if (track_mode == TrackMode::kWide) {
    magnitude = wide::steering_angle_to_curvature(steering_angle_deg);
  } else {
    magnitude = narrow::steering_angle_to_curvature(steering_angle_deg);
  }

  return sign * magnitude;
}

}  // namespace my_robot_maps
