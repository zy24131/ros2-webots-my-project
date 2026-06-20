// L3 转向 / 自转状态机（case 0~5）
//
// case 0~3：通过 SetTrackWidth Action 与 leg_controller 切换轮距
// case 4~5：自转模式，无条件切换（不走 Action）
//
// 外界输入：
//   /my_robot/motion_mode_switch   0=转向  1=顺时针自转  2=逆时针自转
//   /my_robot/track_width_switch   0=窄轮距  1=宽轮距

#include <chrono>
#include <memory>
#include <string>

#include "my_robot_msgs/action/set_track_width.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

namespace {

enum FsmState : int32_t {
  kNarrowTrack = 0,
  kSwitchToWide = 1,
  kWideTrack = 2,
  kSwitchToNarrow = 3,
  kSpinLeft = 4,
  kSpinRight = 5,
};

enum MotionModeSwitch : int32_t {
  kSteering = 0,
  kSpinClockwise = 1,
  kSpinCounterClockwise = 2,
};

enum TrackWidthSwitch : int32_t {
  kTrackNarrow = 0,
  kTrackWide = 1,
};

using SetTrackWidth = my_robot_msgs::action::SetTrackWidth;

const char * ModeName(FsmState state)
{
  switch (state) {
    case kNarrowTrack: return "narrow_track";
    case kSwitchToWide: return "switch_to_wide";
    case kWideTrack: return "wide_track";
    case kSwitchToNarrow: return "switch_to_narrow";
    case kSpinLeft: return "spin_left";
    case kSpinRight: return "spin_right";
    default: return "unknown";
  }
}

}  // namespace

class RobotFsm : public rclcpp::Node
{
public:
  using GoalHandleSetTrackWidth = rclcpp_action::ClientGoalHandle<SetTrackWidth>;

  RobotFsm()
  : Node("robot_fsm")
  {
    state_ = kNarrowTrack;

    mode_pub_ = create_publisher<std_msgs::msg::String>("/my_robot/mode", 10);
    state_pub_ = create_publisher<std_msgs::msg::Int32>("/my_robot/fsm_state", 10);

    motion_mode_sub_ = create_subscription<std_msgs::msg::Int32>(
      "/my_robot/motion_mode_switch", 10,
      [this](const std_msgs::msg::Int32::SharedPtr msg) {
        motion_mode_ = msg->data;
      });

    track_width_sub_ = create_subscription<std_msgs::msg::Int32>(
      "/my_robot/track_width_switch", 10,
      [this](const std_msgs::msg::Int32::SharedPtr msg) {
        track_width_ = msg->data;
      });

    action_client_ = rclcpp_action::create_client<SetTrackWidth>(this, "set_track_width");

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&RobotFsm::Tick, this));

    RCLCPP_INFO(get_logger(), "FSM start: narrow_track (case 0), Action client ready");
  }

private:
  void Tick()
  {
    const FsmState prev = state_;
    UpdateState();
    if (state_ != prev) {
      RCLCPP_INFO(
        get_logger(), "FSM %s -> %s (case %d)",
        ModeName(prev), ModeName(state_), static_cast<int>(state_));
    }
    PublishOutputs();
  }

  void CancelActiveGoal()
  {
    if (!goal_in_flight_ || !active_goal_handle_) {
      goal_in_flight_ = false;
      active_goal_handle_.reset();
      return;
    }
    auto future = action_client_->async_cancel_goal(active_goal_handle_);
    (void)future;
    goal_in_flight_ = false;
    active_goal_handle_.reset();
  }

  void SendTrackWidthGoal(uint8_t target, FsmState switching_state)
  {
    if (!action_client_->wait_for_action_server(std::chrono::milliseconds(0))) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "set_track_width server not ready");
      return;
    }
    if (goal_in_flight_)
      return;

    pending_switching_state_ = switching_state;
    state_ = switching_state;

    auto goal_msg = SetTrackWidth::Goal();
    goal_msg.target = target;

    auto send_options = rclcpp_action::Client<SetTrackWidth>::SendGoalOptions();
    send_options.goal_response_callback =
      [this](const GoalHandleSetTrackWidth::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "Action goal rejected");
          goal_in_flight_ = false;
          state_ = (pending_switching_state_ == kSwitchToWide) ? kNarrowTrack : kWideTrack;
          return;
        }
        active_goal_handle_ = goal_handle;
      };
    send_options.result_callback =
      [this](const GoalHandleSetTrackWidth::WrappedResult & result) {
        OnActionResult(result, pending_switching_state_);
      };

    goal_in_flight_ = true;
    action_client_->async_send_goal(goal_msg, send_options);
  }

  void OnActionResult(
    const GoalHandleSetTrackWidth::WrappedResult & result,
    FsmState switching_state)
  {
    goal_in_flight_ = false;
    active_goal_handle_.reset();

    if (result.code != rclcpp_action::ResultCode::SUCCEEDED ||
      !result.result || !result.result->success)
    {
      RCLCPP_WARN(get_logger(), "SetTrackWidth failed, back to stable state");
      state_ = (switching_state == kSwitchToWide) ? kNarrowTrack : kWideTrack;
      return;
    }

    state_ = (switching_state == kSwitchToWide) ? kWideTrack : kNarrowTrack;
    RCLCPP_INFO(get_logger(), "SetTrackWidth succeeded -> case %d", static_cast<int>(state_));
  }

  void UpdateState()
  {
    // 自转：取消 Action，直接进入 spin 态
    if (motion_mode_ == kSpinClockwise) {
      CancelActiveGoal();
      state_ = kSpinRight;
      return;
    }
    if (motion_mode_ == kSpinCounterClockwise) {
      CancelActiveGoal();
      state_ = kSpinLeft;
      return;
    }

    if (motion_mode_ != kSteering)
      return;

    if (state_ == kSpinLeft || state_ == kSpinRight) {
      state_ = kNarrowTrack;
      return;
    }

    // Action 进行中：等 result_callback 改状态
    if (goal_in_flight_)
      return;

    switch (state_) {
      case kNarrowTrack:
        if (track_width_ == kTrackWide) {
          SendTrackWidthGoal(SetTrackWidth::Goal::TARGET_WIDE, kSwitchToWide);
        }
        break;

      case kSwitchToWide:
        // 等 callback
        break;

      case kWideTrack:
        if (track_width_ == kTrackNarrow) {
          SendTrackWidthGoal(SetTrackWidth::Goal::TARGET_NARROW, kSwitchToNarrow);
        }
        break;

      case kSwitchToNarrow:
        // 等 callback
        break;

      default:
        state_ = kNarrowTrack;
        break;
    }
  }

  void PublishOutputs()
  {
    std_msgs::msg::String mode_msg;
    mode_msg.data = ModeName(state_);
    mode_pub_->publish(mode_msg);

    std_msgs::msg::Int32 state_msg;
    state_msg.data = static_cast<int32_t>(state_);
    state_pub_->publish(state_msg);
  }

  FsmState state_;
  int32_t motion_mode_{kSteering};
  int32_t track_width_{kTrackNarrow};
  bool goal_in_flight_{false};
  FsmState pending_switching_state_{kSwitchToWide};
  std::shared_ptr<GoalHandleSetTrackWidth> active_goal_handle_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr motion_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr track_width_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Client<SetTrackWidth>::SharedPtr action_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotFsm>());
  rclcpp::shutdown();
  return 0;
}
