// FSM 侧 SetTrack Action 客户端（goal 生命周期）

#pragma once

#include <functional>
#include <memory>
#include <string>

#include "fsm/motion.hpp"
#include "fsm/topic_names.hpp"
#include "my_robot_msgs/action/set_track_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace fsm {

class TrackActionClient {
public:
  using GoalHandle = rclcpp_action::ClientGoalHandle<my_robot_msgs::action::SetTrackAction>;

  struct Callbacks {
    std::function<void(RobotMode switching)> on_rejected;
    std::function<void(RobotMode switching, bool success)> on_finished;
  };

  TrackActionClient(
    rclcpp::Node * node,
    const std::string & action_name = topics::kSetTrackAction);

  void SetCallbacks(Callbacks callbacks);

  bool InFlight() const { return in_flight_; }
  RobotMode Switching() const { return switching_; }

  void Cancel();
  bool Request(uint8_t target, RobotMode switching);

private:
  void Clear();
  void OnGoalResponse(const GoalHandle::SharedPtr & goal_handle);
  void OnResult(const GoalHandle::WrappedResult & result);

  rclcpp::Node * node_;
  rclcpp_action::Client<my_robot_msgs::action::SetTrackAction>::SharedPtr client_;
  Callbacks callbacks_;
  bool in_flight_{false};
  RobotMode switching_{kSwitchToWide};
  GoalHandle::SharedPtr handle_;
};

}  // namespace fsm
