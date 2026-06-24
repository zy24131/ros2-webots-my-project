// L2 执行器：SteeringCommand + SetTrackWidth Action → joint_commands（唯一发布者）
// 450ms 命令看门狗：超时归零

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>

#include "my_robot_msgs/action/set_track_width.hpp"
#include "my_robot_msgs/msg/steering_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace {

struct JointDef {
  const char * name;
  bool is_wheel_arm;
};

constexpr std::array<JointDef, 8> kJoints = {{
  {"link_002_joint", false},
  {"link_003_joint", true},
  {"link_005_joint", false},
  {"link_006_joint", true},
  {"link_008_joint", false},
  {"link_009_joint", true},
  {"link_011_joint", false},
  {"link_012_joint", true},
}};

using SetTrackWidth = my_robot_msgs::action::SetTrackWidth;
using GoalHandleSetTrackWidth = rclcpp_action::ServerGoalHandle<SetTrackWidth>;

}  // namespace

class ActuatorExecutorNode : public rclcpp::Node
{
public:
  ActuatorExecutorNode()
  : Node("actuator_executor")
  {
    declare_parameter("position_tolerance", 0.05);
    declare_parameter("wide_neutral_angle_deg", 45.0);
    declare_parameter("command_timeout_ms", 450);

    cmd_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "/my_robot/joint_commands", 10);
    ready_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/my_robot/system_ready", 10);

    state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/my_robot/joint_states", 10,
      std::bind(&ActuatorExecutorNode::JointStateCallback, this, std::placeholders::_1));

    steering_sub_ = create_subscription<my_robot_msgs::msg::SteeringCommand>(
      "/my_robot/steering_command", 10,
      [this](const my_robot_msgs::msg::SteeringCommand::SharedPtr msg) {
        last_steering_cmd_ = *msg;
        last_steering_time_ = now();
        has_steering_cmd_ = true;
      });

    action_server_ = rclcpp_action::create_server<SetTrackWidth>(
      this,
      "set_track_width",
      std::bind(&ActuatorExecutorNode::HandleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ActuatorExecutorNode::HandleCancel, this, std::placeholders::_1),
      std::bind(&ActuatorExecutorNode::HandleAccepted, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&ActuatorExecutorNode::Tick, this));

    RCLCPP_INFO(get_logger(), "actuator_executor: sole joint_commands publisher + watchdog");
  }

private:
  void JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (i >= msg->position.size()) {
        break;
      }
      current_positions_[msg->name[i]] = msg->position[i];
    }
    if (!initialized_ && current_positions_.size() >= 8) {
      initialized_ = true;
      std_msgs::msg::Bool ready;
      ready.data = true;
      ready_pub_->publish(ready);
      RCLCPP_INFO(get_logger(), "joint_states ready, system_ready=true");
    }
  }

  double JointAngleForActionTarget(uint8_t target) const
  {
    if (target == SetTrackWidth::Goal::TARGET_WIDE) {
      return get_parameter("wide_neutral_angle_deg").as_double() * M_PI / 180.0;
    }
    return 0.0;
  }

  void BuildActionTargets(uint8_t target, std::unordered_map<std::string, double> & out) const
  {
    out.clear();
    for (const JointDef & joint : kJoints) {
      out[joint.name] = JointAngleForActionTarget(target);
    }
  }

  rclcpp_action::GoalResponse HandleGoal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const SetTrackWidth::Goal> goal)
  {
    if (goal->target != SetTrackWidth::Goal::TARGET_NARROW &&
      goal->target != SetTrackWidth::Goal::TARGET_WIDE)
    {
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (active_goal_) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse HandleCancel(
    const std::shared_ptr<GoalHandleSetTrackWidth> /*goal_handle*/)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void HandleAccepted(const std::shared_ptr<GoalHandleSetTrackWidth> goal_handle)
  {
    active_goal_ = goal_handle;
    action_target_ = goal_handle->get_goal()->target;
    BuildActionTargets(action_target_, command_positions_);
    initial_max_error_ = ComputeMaxError(command_positions_);
    const double tolerance = get_parameter("position_tolerance").as_double();
    if (initial_max_error_ < tolerance) {
      initial_max_error_ = 1.0;
    }
  }

  void Tick()
  {
    if (!initialized_) {
      return;
    }
    if (active_goal_) {
      ExecuteAction();
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
    std::array<double, 8> positions{};
    if (SteeringCommandStale() || !last_steering_cmd_.valid) {
      PublishJointCommand(positions);
      return;
    }
    for (size_t i = 0; i < positions.size(); ++i) {
      positions[i] = last_steering_cmd_.joint_positions[i];
    }
    PublishJointCommand(positions);
  }

  double ComputeMaxError(const std::unordered_map<std::string, double> & target) const
  {
    double max_err = 0.0;
    for (const auto & entry : target) {
      const auto it = current_positions_.find(entry.first);
      if (it == current_positions_.end()) {
        continue;
      }
      max_err = std::max(max_err, std::abs(it->second - entry.second));
    }
    return max_err;
  }

  void PublishJointCommand(const std::array<double, 8> & positions)
  {
    sensor_msgs::msg::JointState cmd;
    cmd.header.stamp = now();
    for (size_t i = 0; i < kJoints.size(); ++i) {
      cmd.name.push_back(kJoints[i].name);
      cmd.position.push_back(positions[i]);
    }
    cmd_pub_->publish(cmd);
  }

  void ExecuteAction()
  {
    if (active_goal_->is_canceling()) {
      auto result = std::make_shared<SetTrackWidth::Result>();
      result->success = false;
      result->message = "canceled";
      active_goal_->canceled(result);
      active_goal_.reset();
      return;
    }

    const double tolerance = get_parameter("position_tolerance").as_double();
    std::unordered_map<std::string, double> cmd = command_positions_;

    for (auto & entry : cmd) {
      const auto it = current_positions_.find(entry.first);
      if (it == current_positions_.end()) {
        continue;
      }
      entry.second = it->second + 0.15 * (entry.second - it->second);
    }

    std::array<double, 8> positions{};
    for (size_t i = 0; i < kJoints.size(); ++i) {
      const auto it = cmd.find(kJoints[i].name);
      if (it != cmd.end()) {
        positions[i] = it->second;
      }
    }
    PublishJointCommand(positions);

    const double max_err = ComputeMaxError(command_positions_);
    auto feedback = std::make_shared<SetTrackWidth::Feedback>();
    feedback->progress = static_cast<float>(
      std::clamp(1.0 - max_err / initial_max_error_, 0.0, 1.0));
    active_goal_->publish_feedback(feedback);

    if (max_err <= tolerance) {
      std::array<double, 8> final_positions{};
      for (size_t i = 0; i < kJoints.size(); ++i) {
        const auto it = command_positions_.find(kJoints[i].name);
        if (it != command_positions_.end()) {
          final_positions[i] = it->second;
        }
      }
      PublishJointCommand(final_positions);
      auto result = std::make_shared<SetTrackWidth::Result>();
      result->success = true;
      result->message = "reached target";
      active_goal_->succeed(result);
      active_goal_.reset();
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Subscription<my_robot_msgs::msg::SteeringCommand>::SharedPtr steering_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Server<SetTrackWidth>::SharedPtr action_server_;

  std::unordered_map<std::string, double> current_positions_;
  std::unordered_map<std::string, double> command_positions_;
  std::shared_ptr<GoalHandleSetTrackWidth> active_goal_;
  uint8_t action_target_{0};
  double initial_max_error_{1.0};

  my_robot_msgs::msg::SteeringCommand last_steering_cmd_;
  rclcpp::Time last_steering_time_;
  bool has_steering_cmd_{false};
  bool initialized_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ActuatorExecutorNode>());
  rclcpp::shutdown();
  return 0;
}
