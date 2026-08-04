// SetTrack Action 执行体：插值趋近 + 进度/到位判定（无 ROS 依赖）

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include "fsm/joint_config.hpp"
#include "my_robot_msgs/action/set_track_action.hpp"

namespace actuator_control {

inline std::array<double, fsm::kSteeringJointCount> TrackTargetPositions(
  uint8_t target,
  double wide_neutral_rad)
{
  const double angle =
    target == my_robot_msgs::action::SetTrackAction::Goal::TARGET_WIDE ?
    wide_neutral_rad :
    0.0;
  std::array<double, fsm::kSteeringJointCount> positions{};
  positions.fill(angle);
  return positions;
}

inline double MaxJointError(
  const std::array<double, fsm::kSteeringJointCount> & current,
  const std::array<double, fsm::kSteeringJointCount> & target)
{
  double max_err = 0.0;
  for (size_t i = 0; i < fsm::kSteeringJointCount; ++i) {
    max_err = std::max(max_err, std::abs(current[i] - target[i]));
  }
  return max_err;
}

class TrackTask {
public:
  void Configure(double step_ratio, double min_initial_error)
  {
    step_ratio_ = step_ratio;
    min_initial_error_ = min_initial_error;
  }

  void Begin(
    const std::array<double, fsm::kSteeringJointCount> & target,
    const std::array<double, fsm::kSteeringJointCount> & current,
    double tolerance)
  {
    target_ = target;                           // 保存目标位置
    active_ = true;                             // 激活任务
    initial_error_ = MaxJointError(current, target_);  // 记录初始最大误差
    if (initial_error_ < tolerance) {           // 若已在容差范围内，避免零除
      initial_error_ = min_initial_error_;
    }
  }

  bool Active() const { return active_; }

  void Clear() { active_ = false; }

  float Step(
    const std::array<double, fsm::kSteeringJointCount> & current,
    std::array<double, fsm::kSteeringJointCount> & out_cmd) const
  {
    for (size_t i = 0; i < fsm::kSteeringJointCount; ++i) {
      out_cmd[i] = current[i] + step_ratio_ * (target_[i] - current[i]);
    }
    const double max_err = MaxJointError(current, target_);
    return static_cast<float>(
      std::clamp(1.0 - max_err / initial_error_, 0.0, 1.0));
  }

  bool Reached(const std::array<double, fsm::kSteeringJointCount> & current, double tolerance) const
  {
    return MaxJointError(current, target_) <= tolerance;
  }

  const std::array<double, fsm::kSteeringJointCount> & Target() const { return target_; }

private:
  std::array<double, fsm::kSteeringJointCount> target_{};
  double initial_error_{1.0};
  double step_ratio_{0.15};
  double min_initial_error_{1.0};
  bool active_{false};
};

}  // namespace actuator_control
