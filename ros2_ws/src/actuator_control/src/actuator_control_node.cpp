// L2 执行器：SteeringCommand + SetTrack Action → joint_commands（唯一发布者）
// 450ms 命令看门狗：超时归零

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "actuator_control/track_task.hpp"
#include "fsm/joint_config.hpp"
#include "fsm/qos.hpp"
#include "fsm/topic_names.hpp"
#include "my_robot_msgs/action/set_track_action.hpp"
#include "my_robot_msgs/msg/steering_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace {

using my_robot_msgs::action::SetTrackAction;
using GoalHandleSetTrackAction = rclcpp_action::ServerGoalHandle<SetTrackAction>;

}  // namespace

class ActuatorControlNode : public rclcpp::Node
{
public:
  ActuatorControlNode()
  : Node("actuator_control")
  {
    declare_parameter("position_tolerance", 0.05);
    declare_parameter("wide_neutral_angle_deg", 45.0);
    declare_parameter("command_timeout_ms", 450);
    declare_parameter("track_step_ratio", 0.15);

    track_task_.Configure(
      get_parameter("track_step_ratio").as_double(),
      1.0);

    cmd_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      fsm::topics::kJointCommands,
      fsm::QoSFromParams(this, "qos.joint_commands_pub", "reliable"));

    state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      fsm::topics::kJointStates,
      fsm::QoSFromParams(this, "qos.joint_states_sub", "sensor_data"),
      std::bind(&ActuatorControlNode::JointStateCallback, this, std::placeholders::_1));

    steering_sub_ = create_subscription<my_robot_msgs::msg::SteeringCommand>(
      fsm::topics::kSteeringCommand,
      fsm::QoSFromParams(this, "qos.steering_command_sub", "reliable"),
      [this](const my_robot_msgs::msg::SteeringCommand::SharedPtr msg) {
        last_steering_cmd_ = *msg;
        last_steering_time_ = now();
        has_steering_cmd_ = true;
      });

    action_server_ = rclcpp_action::create_server<SetTrackAction>(
      this,
      fsm::topics::kSetTrackAction,
      std::bind(&ActuatorControlNode::HandleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ActuatorControlNode::HandleCancel, this, std::placeholders::_1),
      std::bind(&ActuatorControlNode::HandleAccepted, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&ActuatorControlNode::Tick, this));

    RCLCPP_INFO(get_logger(), "actuator_control: sole joint_commands publisher + watchdog");
  }

private:
  void JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (i >= msg->position.size()) {
        break;
      }
      for (size_t j = 0; j < fsm::kSteeringJoints.size(); ++j) {
        if (msg->name[i] == fsm::kSteeringJoints[j].name) {
          current_positions_[j] = msg->position[i];
          joint_seen_[j] = true;
        }
      }
    }
    if (!initialized_ &&
      std::all_of(joint_seen_.begin(), joint_seen_.end(), [](bool seen) {return seen;}))
    {
      initialized_ = true;
      RCLCPP_INFO(get_logger(), "joint_states ready");
    }
  }

  double WideNeutralRad() const
  {
    return get_parameter("wide_neutral_angle_deg").as_double() * M_PI / 180.0;
  }

  rclcpp_action::GoalResponse HandleGoal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const SetTrackAction::Goal> goal)
  {
    if (goal->target != SetTrackAction::Goal::TARGET_NARROW &&
      goal->target != SetTrackAction::Goal::TARGET_WIDE)
    {
      return rclcpp_action::GoalResponse::REJECT;
    }
    
    //就是判断是否已经有目标在执行了，如果有的话就拒绝新的目标。
    if (active_goal_ || track_task_.Active()) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse HandleCancel(
    const std::shared_ptr<GoalHandleSetTrackAction> /*goal_handle*/)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void HandleAccepted(const std::shared_ptr<GoalHandleSetTrackAction> goal_handle)
  {
    active_goal_ = goal_handle;
    const auto target = actuator_control::TrackTargetPositions(
      goal_handle->get_goal()->target, WideNeutralRad());
    track_task_.Begin(
      target,
      current_positions_,
      get_parameter("position_tolerance").as_double());
  }

  void Tick()
  {
    if (!initialized_) {
      return;
    }
    if (active_goal_) {
      ExecuteTrackAction();
      return;
    }
    PublishFromSteeringCommand();
  }

  bool SteeringCommandStale() const
  {
    if (!has_steering_cmd_) {
      return true;
    }
    const int timeout_ms = get_parameter("command_timeout_ms").as_int();
    const auto age = now() - last_steering_time_;
    return age > std::chrono::milliseconds(timeout_ms);
  }

  void PublishFromSteeringCommand()
  {
    std::array<double, fsm::kSteeringJointCount> positions{};
    if (!SteeringCommandStale() && last_steering_cmd_.valid) {
      for (size_t i = 0; i < positions.size(); ++i) {
        positions[i] = last_steering_cmd_.joint_positions[i];
      }
    }
    PublishJointCommand(positions);
  }

  void PublishJointCommand(const std::array<double, fsm::kSteeringJointCount> & positions)
  {
    sensor_msgs::msg::JointState cmd;
    cmd.header.stamp = now();
    for (size_t i = 0; i < fsm::kSteeringJoints.size(); ++i) {
      cmd.name.push_back(fsm::kSteeringJoints[i].name);
      cmd.position.push_back(positions[i]);
    }
    cmd_pub_->publish(cmd);
  }


  /*ExecuteTrackAction() 每20ms执行一次:
  ↓
  track_task_.Step() → 计算下一步关节位置
    ↓
  PublishJointCommand(cmd) → 发送关节指令到执行器
    ↓
  publish_feedback(progress) → 通知客户端当前进度
    ↓
  track_task_.Reached()? → 到达目标?
    ↓
  succeed(result) → 返回成功给客户端
        ↓
  客户端 OnResult() 被调用 → 触发 callbacks_.on_finished*/

  void ExecuteTrackAction()
  {
    if (active_goal_->is_canceling()) {
      auto result = std::make_shared<SetTrackAction::Result>();
      result->success = false;
      result->message = "canceled";
      active_goal_->canceled(result);
      active_goal_.reset();
      track_task_.Clear();
      return;
    }

    const double tolerance = get_parameter("position_tolerance").as_double();
    std::array<double, fsm::kSteeringJointCount> cmd{};
    const float progress = track_task_.Step(current_positions_, cmd);
    PublishJointCommand(cmd);

    auto feedback = std::make_shared<SetTrackAction::Feedback>();
    feedback->progress = progress;
    active_goal_->publish_feedback(feedback);

    if (track_task_.Reached(current_positions_, tolerance)) {
      PublishJointCommand(track_task_.Target());
      auto result = std::make_shared<SetTrackAction::Result>();
      result->success = true;
      result->message = "reached target";
      active_goal_->succeed(result);
      active_goal_.reset();
      track_task_.Clear();
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Subscription<my_robot_msgs::msg::SteeringCommand>::SharedPtr steering_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Server<SetTrackAction>::SharedPtr action_server_;

  std::array<double, fsm::kSteeringJointCount> current_positions_{};
  std::array<bool, fsm::kSteeringJointCount> joint_seen_{};
  std::shared_ptr<GoalHandleSetTrackAction> active_goal_;
  actuator_control::TrackTask track_task_;

  my_robot_msgs::msg::SteeringCommand last_steering_cmd_;
  rclcpp::Time last_steering_time_;
  bool has_steering_cmd_{false};
  bool initialized_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ActuatorControlNode>());
  rclcpp::shutdown();
  return 0;
}
