#include "fsm/track_action_client.hpp"

namespace fsm {

TrackActionClient::TrackActionClient(rclcpp::Node * node, const std::string & action_name)
: node_(node),
  client_(rclcpp_action::create_client<my_robot_msgs::action::SetTrackAction>(node, action_name))
{
}

void TrackActionClient::SetCallbacks(Callbacks callbacks)
{
  callbacks_ = std::move(callbacks);
}

void TrackActionClient::Clear()
{
  in_flight_ = false;
  handle_.reset();
}

void TrackActionClient::Cancel()
{
  if (in_flight_ && handle_) {
    (void)client_->async_cancel_goal(handle_);
  }
  Clear();
}

bool TrackActionClient::Request(uint8_t target, RobotMode switching)
{
  if (!client_->wait_for_action_server(std::chrono::milliseconds(0))) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "set_track server not ready");
    return false;
  }
  if (in_flight_) {
    return false;
  }

  switching_ = switching;
  in_flight_ = true;

  my_robot_msgs::action::SetTrackAction::Goal goal;
  goal.target = target;

  auto opts = rclcpp_action::Client<my_robot_msgs::action::SetTrackAction>::SendGoalOptions();
  opts.goal_response_callback =
    [this](const GoalHandle::SharedPtr & goal_handle) { OnGoalResponse(goal_handle); };
  opts.result_callback =
    [this](const GoalHandle::WrappedResult & result) { OnResult(result); };

  client_->async_send_goal(goal, opts);
  return true;
}

void TrackActionClient::OnGoalResponse(const GoalHandle::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "SetTrack goal rejected");
    const RobotMode switching = switching_;
    Clear();
    if (callbacks_.on_rejected) {
      callbacks_.on_rejected(switching);
    }
    return;
  }
  handle_ = goal_handle;
}

void TrackActionClient::OnResult(const GoalHandle::WrappedResult & result)
{
  const RobotMode switching = switching_;
  const bool success =
    result.code == rclcpp_action::ResultCode::SUCCEEDED &&
    result.result && result.result->success;
  Clear();
  if (callbacks_.on_finished) {
    callbacks_.on_finished(switching, success);
  }
}

}  // namespace fsm
