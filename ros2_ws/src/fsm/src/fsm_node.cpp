// L3 转向 / 自转状态机（case 0~5），显式 Trigger 驱动

#include <chrono>
#include <memory>
#include <string>

#include "fsm/motion_mode.hpp"
#include "fsm/qos.hpp"
#include "my_robot_msgs/action/set_track_width.hpp"
#include "my_robot_msgs/srv/set_motion_mode.hpp"
#include "my_robot_msgs/srv/set_track_width_switch.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

using namespace fsm;

// 定义 Action / Service 类型
using SetTrackWidth = my_robot_msgs::action::SetTrackWidth;
using SetMotionMode = my_robot_msgs::srv::SetMotionMode;
using SetTrackWidthSwitch = my_robot_msgs::srv::SetTrackWidthSwitch;

class RobotFsm : public rclcpp::Node
{
public:
  // 定义 Action 句柄类型
  using GoalHandleSetTrackWidth = rclcpp_action::ClientGoalHandle<SetTrackWidth>;

  RobotFsm()
  : Node("fsm")
  {
    // 初始化状态为窄轮距
    state_ = kNarrowTrack;

    // 创建模式发布者
    mode_pub_ = create_publisher<std_msgs::msg::String>(
      "/my_robot/mode",
      fsm::QoSFromParams(this, "qos.mode_pub", "reliable"));
    state_pub_ = create_publisher<std_msgs::msg::Int32>(
      "/my_robot/fsm_state",
      fsm::QoSFromParams(this, "qos.fsm_state_pub", "reliable"));

    // 模式切换 Service（替代 motion_mode_switch / track_width_switch 话题）
    motion_mode_srv_ = create_service<SetMotionMode>(
      "/my_robot/set_motion_mode",
      std::bind(&RobotFsm::HandleSetMotionMode, this, std::placeholders::_1, std::placeholders::_2));

    track_width_srv_ = create_service<SetTrackWidthSwitch>(
      "/my_robot/set_track_width_switch",
      std::bind(
        &RobotFsm::HandleSetTrackWidthSwitch, this,
        std::placeholders::_1, std::placeholders::_2));

    // 创建轮距切换 Action 客户端
    action_client_ = rclcpp_action::create_client<SetTrackWidth>(this, "set_track_width");

    // 创建定时器
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&RobotFsm::Tick, this));

    RCLCPP_INFO(get_logger(), "FSM start: narrow_track (case 0), service + trigger-driven");
  }

private:
  /**
   * @brief 处理运动模式设置请求
   * 
   * 验证请求的运动模式是否有效（0=转向模式，1=顺时针旋转，2=逆时针旋转），
   * 若有效则更新内部运动模式状态，并返回设置结果。
   * 
   * @param request 包含目标运动模式的服务请求
   * @param response 返回设置是否成功及相应消息
   */
  void HandleSetMotionMode(
    const std::shared_ptr<SetMotionMode::Request> request,
    std::shared_ptr<SetMotionMode::Response> response)
  {
    if (!IsValidMotionIntent(request->mode)) {
      response->success = false;
      response->message = "invalid mode: use 0=steering 1=spin_cw 2=spin_ccw";
      return;
    }
    motion_mode_ = static_cast<MotionIntent>(request->mode);
    response->success = true;
    response->message = "motion_mode=" + std::to_string(motion_mode_);
    RCLCPP_INFO(get_logger(), "SetMotionMode: %s", response->message.c_str());
  }

  void HandleSetTrackWidthSwitch(
    const std::shared_ptr<SetTrackWidthSwitch::Request> request,
    std::shared_ptr<SetTrackWidthSwitch::Response> response)
  {
    if (!IsValidTrackWidthIntent(request->track_width)) {
      response->success = false;
      response->message = "invalid track_width: use 0=narrow 1=wide";
      return;
    }
    track_width_ = static_cast<TrackWidthIntent>(request->track_width);
    response->success = true;
    response->message = "track_width=" + std::to_string(track_width_);
    RCLCPP_INFO(get_logger(), "SetTrackWidthSwitch: %s", response->message.c_str());
  }

  // 检测触发器
  FsmTrigger DetectTrigger() const
  {
    if (motion_mode_ == kSpinClockwise) {
      return FsmTrigger::kSpinClockwise;
    }
    if (motion_mode_ == kSpinCounterClockwise) {
      return FsmTrigger::kSpinCounterClockwise;
    }
    if (motion_mode_ != kSteering) {
      return FsmTrigger::kNone;
    }
    if (state_ == kSpinLeft || state_ == kSpinRight) {
      return FsmTrigger::kReturnFromSpin;
    }
    if (track_width_goal_.in_flight) {
      return FsmTrigger::kNone;
    }
    if (state_ == kNarrowTrack && track_width_ == kTrackWide) {
      return FsmTrigger::kRequestWide;
    }
    if (state_ == kWideTrack && track_width_ == kTrackNarrow) {
      return FsmTrigger::kRequestNarrow;
    }
    return FsmTrigger::kNone;
  }

  // 根据触发器执行相应的动作
  void FireTrigger(FsmTrigger trigger)
  {
    switch (trigger) {
      case FsmTrigger::kSpinClockwise:
        CancelTrackWidthGoal();
        state_ = kSpinRight;
        return;
      case FsmTrigger::kSpinCounterClockwise:
        CancelTrackWidthGoal();
        state_ = kSpinLeft;
        return;
      case FsmTrigger::kReturnFromSpin:
        state_ = kNarrowTrack;
        return;
      case FsmTrigger::kRequestWide:
        RequestTrackWidth(SetTrackWidth::Goal::TARGET_WIDE, kSwitchToWide);
        return;
      case FsmTrigger::kRequestNarrow:
        RequestTrackWidth(SetTrackWidth::Goal::TARGET_NARROW, kSwitchToNarrow);
        return;
      default:
        return;
    }
  }

  // 主循环：检测触发器、状态转换、发布输出
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

  struct TrackWidthGoal {
    bool in_flight{false};
    RobotMode switching{kSwitchToWide};
    GoalHandleSetTrackWidth::SharedPtr handle;
  };

  void ClearTrackWidthGoal()
  {
    track_width_goal_.in_flight = false;
    track_width_goal_.handle.reset();
  }

  void CancelTrackWidthGoal()
  {
    if (track_width_goal_.in_flight && track_width_goal_.handle) {
      (void)action_client_->async_cancel_goal(track_width_goal_.handle);
    }
    ClearTrackWidthGoal();
  }

  void RequestTrackWidth(uint8_t target, RobotMode switching)
  {
    if (!action_client_->wait_for_action_server(std::chrono::milliseconds(0))) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "set_track_width server not ready");
      return;
    }
    if (track_width_goal_.in_flight) {
      return;
    }

    track_width_goal_.switching = switching;
    track_width_goal_.in_flight = true;
    state_ = switching;

    SetTrackWidth::Goal goal;
    goal.target = target;

    auto opts = rclcpp_action::Client<SetTrackWidth>::SendGoalOptions();
    opts.goal_response_callback =
      std::bind(&RobotFsm::OnTrackWidthGoalResponse, this, std::placeholders::_1);
    opts.result_callback =
      std::bind(&RobotFsm::OnTrackWidthGoalResult, this, std::placeholders::_1);

    action_client_->async_send_goal(goal, opts);
  }

  void OnTrackWidthGoalResponse(const GoalHandleSetTrackWidth::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "set_track_width goal rejected");
      state_ = StableTrackBeforeSwitch(track_width_goal_.switching);
      ClearTrackWidthGoal();
      return;
    }
    track_width_goal_.handle = goal_handle;
  }

  void OnTrackWidthGoalResult(const GoalHandleSetTrackWidth::WrappedResult & result)
  {
    const RobotMode switching = track_width_goal_.switching;
    ClearTrackWidthGoal();

    if (result.code != rclcpp_action::ResultCode::SUCCEEDED ||
      !result.result || !result.result->success)
    {
      RCLCPP_WARN(get_logger(), "set_track_width failed, revert to stable");
      state_ = StableTrackBeforeSwitch(switching);
      return;
    }

    state_ = StableTrackAfterSwitch(switching);
    RCLCPP_INFO(get_logger(), "set_track_width ok -> %s", ModeToString(state_));
  }

  // 发布输出
  void PublishOutputs()
  {
    std_msgs::msg::String mode_msg;
    mode_msg.data = ModeToString(state_);
    mode_pub_->publish(mode_msg);

    std_msgs::msg::Int32 state_msg;
    state_msg.data = static_cast<int32_t>(state_);
    state_pub_->publish(state_msg);
  }

  RobotMode state_;
  MotionIntent motion_mode_{kSteering};
  TrackWidthIntent track_width_{kTrackNarrow};
  TrackWidthGoal track_width_goal_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_pub_;
  rclcpp::Service<SetMotionMode>::SharedPtr motion_mode_srv_;
  rclcpp::Service<SetTrackWidthSwitch>::SharedPtr track_width_srv_;
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
