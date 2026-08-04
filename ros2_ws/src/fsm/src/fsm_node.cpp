// L3 转向 / 自转状态机（case 0~5），显式 Trigger 驱动

#include <chrono>
#include <memory>
#include <string>

#include "fsm/motion.hpp"
#include "fsm/qos.hpp"
#include "fsm/topic_names.hpp"
#include "fsm/track_action_client.hpp"
#include "my_robot_msgs/action/set_track_action.hpp"
#include "my_robot_msgs/srv/set_motion.hpp"
#include "my_robot_msgs/srv/set_track.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace fsm;

using SetMotion = my_robot_msgs::srv::SetMotion;
using SetTrack = my_robot_msgs::srv::SetTrack;
using SetTrackAction = my_robot_msgs::action::SetTrackAction;

class RobotFsm : public rclcpp::Node
{
public:
  RobotFsm()
  : Node("fsm"),
    track_client_(this)
  {
    state_ = kNarrowTrack;
    motion_ = kMotionSteering;
    track_intent_ = kTrackNarrow;

    mode_pub_ = create_publisher<std_msgs::msg::String>(
      topics::kMode,
      fsm::QoSFromParams(this, "qos.mode_pub", "reliable"));

    motion_srv_ = create_service<SetMotion>(
      topics::kSetMotion,
      std::bind(&RobotFsm::HandleSetMotion, this, std::placeholders::_1, std::placeholders::_2));

    track_srv_ = create_service<SetTrack>(
      topics::kSetTrack,
      std::bind(
        &RobotFsm::HandleSetTrack, this,
        std::placeholders::_1, std::placeholders::_2));

    track_client_.SetCallbacks({
      [this](RobotMode switching) {
        state_ = StableTrackBeforeSwitch(switching);
      },
      [this](RobotMode switching, bool success) {
        if (!success) {
          RCLCPP_WARN(get_logger(), "SetTrack failed, revert to stable");
          state_ = StableTrackBeforeSwitch(switching);
          return;
        }
        state_ = StableTrackAfterSwitch(switching);
        RCLCPP_INFO(get_logger(), "SetTrack ok -> %s", ModeToString(state_));
      },
    });

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&RobotFsm::Tick, this));

    RCLCPP_INFO(get_logger(), "FSM start: narrow_track (case 0), Motion/Track service + trigger-driven");
  }

private:
  void HandleSetMotion(
    const std::shared_ptr<SetMotion::Request> request,
    std::shared_ptr<SetMotion::Response> response)
  {
    if (!IsValidMotion(request->mode)) {
      response->success = false;
      response->message = "invalid mode: use 0=steering 1=spin_cw 2=spin_ccw";
      return;
    }
    motion_ = static_cast<Motion>(request->mode);
    response->success = true;
    response->message = "motion=" + std::to_string(motion_);
    RCLCPP_INFO(get_logger(), "SetMotion: %s", response->message.c_str());
  }

  void HandleSetTrack(
    const std::shared_ptr<SetTrack::Request> request,
    std::shared_ptr<SetTrack::Response> response)
  {
    if (!IsValidTrackIntent(request->track)) {
      response->success = false;
      response->message = "invalid track: use 0=narrow 1=wide";
      return;
    }
    track_intent_ = static_cast<TrackIntent>(request->track);
    response->success = true;
    response->message = "track=" + std::to_string(track_intent_);
    RCLCPP_INFO(get_logger(), "SetTrack: %s", response->message.c_str());
  }

  FsmTrigger DetectTrigger() const
  {
    if (motion_ == kMotionSpinClockwise) {
      return FsmTrigger::kMotionSpinClockwise;
    }
    if (motion_ == kMotionSpinCounterClockwise) {
      return FsmTrigger::kMotionSpinCounterClockwise;
    }
    if (motion_ != kMotionSteering) {
      return FsmTrigger::kNone;
    }
    if (state_ == kSpinLeft || state_ == kSpinRight) {
      return FsmTrigger::kMotionReturnFromSpin;
    }
    if (track_client_.InFlight()) {
      return FsmTrigger::kNone;
    }
    if (state_ == kNarrowTrack && track_intent_ == kTrackWide) {
      return FsmTrigger::kTrackRequestWide;
    }
    if (state_ == kWideTrack && track_intent_ == kTrackNarrow) {
      return FsmTrigger::kTrackRequestNarrow;
    }
    return FsmTrigger::kNone;
  }

  void FireTrigger(FsmTrigger trigger)
  {
    switch (trigger) {
      case FsmTrigger::kMotionSpinClockwise:
        track_client_.Cancel();
        state_ = kSpinRight;
        return;
      case FsmTrigger::kMotionSpinCounterClockwise:
        track_client_.Cancel();
        state_ = kSpinLeft;
        return;
      case FsmTrigger::kMotionReturnFromSpin:
        state_ = kNarrowTrack;
        return;
      case FsmTrigger::kTrackRequestWide:
        if (track_client_.Request(SetTrackAction::Goal::TARGET_WIDE, kSwitchToWide)) {
          state_ = kSwitchToWide;
        }
        return;
      case FsmTrigger::kTrackRequestNarrow:
        if (track_client_.Request(SetTrackAction::Goal::TARGET_NARROW, kSwitchToNarrow)) {
          state_ = kSwitchToNarrow;
        }
        return;
      default:
        return;
    }
  }

  void Tick()
  {
    const RobotMode prev = state_;
    const FsmTrigger trigger = DetectTrigger();
    if (trigger != FsmTrigger::kNone) {
      FireTrigger(trigger);
    }
    if (state_ != prev) {
      RCLCPP_INFO(
        get_logger(), "FSM %s -> %s (case %d)",
        ModeToString(prev), ModeToString(state_), static_cast<int>(state_));
    }
    PublishOutputs();
  }

  void PublishOutputs()
  {
    std_msgs::msg::String mode_msg;
    mode_msg.data = ModeToString(state_);
    mode_pub_->publish(mode_msg);
  }

  RobotMode state_;
  Motion motion_;
  TrackIntent track_intent_;
  TrackActionClient track_client_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Service<SetMotion>::SharedPtr motion_srv_;
  rclcpp::Service<SetTrack>::SharedPtr track_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotFsm>());
  rclcpp::shutdown();
  return 0;
}
