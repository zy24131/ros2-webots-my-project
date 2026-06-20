// L2 腿关节控制 + SetTrackWidth Action Server
//
// narrow/wide_track：订阅曲率，经 map() 生成 8 关节角
// spin_left/spin_right：map_spin()，节臂 0、轮臂对角偏转
// Action 轮距切换：JointAngleForMode

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>

#include "my_robot_maps/curvature_to_joints.hpp"
#include "my_robot_maps/spin_to_joints.hpp"
#include "my_robot_msgs/action/set_track_width.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

namespace {

enum class JointRole { kSectionArm, kWheelArm };

struct JointDef {
  const char * name;
  int position;
  JointRole role;
  const char * label;
};

constexpr std::array<JointDef, 8> kJoints = {{
  {"link_002_joint", 1, JointRole::kSectionArm, "右前节臂"},
  {"link_003_joint", 2, JointRole::kWheelArm, "右前轮臂"},
  {"link_005_joint", 3, JointRole::kSectionArm, "左前节臂"},
  {"link_006_joint", 4, JointRole::kWheelArm, "左前轮臂"},
  {"link_008_joint", 5, JointRole::kSectionArm, "左后节臂"},
  {"link_009_joint", 6, JointRole::kWheelArm, "右后轮臂"},
  {"link_011_joint", 7, JointRole::kSectionArm, "右后节臂"},
  {"link_012_joint", 8, JointRole::kWheelArm, "左后轮臂"},
}};

enum class SteeringMode {
  kNarrowTrack,
  kWideTrack,
  kSpinLeft,
  kSpinRight,
  kUnknown,
};

using SetTrackWidth = my_robot_msgs::action::SetTrackWidth;
using GoalHandleSetTrackWidth = rclcpp_action::ServerGoalHandle<SetTrackWidth>;

SteeringMode ModeFromString(const std::string & mode)
{
  if (mode == "narrow_track" || mode == "switch_to_narrow") {
    return SteeringMode::kNarrowTrack;
  }
  if (mode == "wide_track" || mode == "switch_to_wide") {
    return SteeringMode::kWideTrack;
  }
  if (mode == "spin_left") {
    return SteeringMode::kSpinLeft;
  }
  if (mode == "spin_right") {
    return SteeringMode::kSpinRight;
  }
  return SteeringMode::kUnknown;
}

my_robot_maps::TrackMode ToTrackMode(SteeringMode mode)
{
  return mode == SteeringMode::kWideTrack ?
         my_robot_maps::TrackMode::kWide :
         my_robot_maps::TrackMode::kNarrow;
}

bool UsesCurvatureMap(SteeringMode mode)
{
  return mode == SteeringMode::kNarrowTrack || mode == SteeringMode::kWideTrack;
}

bool UsesSpinMap(SteeringMode mode)
{
  return mode == SteeringMode::kSpinLeft || mode == SteeringMode::kSpinRight;
}

my_robot_maps::SpinDirection ToSpinDirection(SteeringMode mode)
{
  return mode == SteeringMode::kSpinLeft ?
         my_robot_maps::SpinDirection::kLeft :
         my_robot_maps::SpinDirection::kRight;
}

}  // namespace

class LegController : public rclcpp::Node
{
public:
  LegController()
  : Node("leg_controller")
  {
    declare_parameter("position_tolerance", 0.05);
    declare_parameter("wide_steering_angle", 0.5);

    cmd_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "/my_robot/joint_commands", 10);
    state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/my_robot/joint_states", 10,
      std::bind(&LegController::JointStateCallback, this, std::placeholders::_1));
    mode_sub_ = create_subscription<std_msgs::msg::String>(
      "/my_robot/mode", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) { mode_ = msg->data; });
    curvature_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/my_robot/steering_curvature", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        steering_curvature_ = msg->data;
      });
    steering_wheel_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/my_robot/steering_wheel", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        steering_angle_deg_ = msg->data;
      });

    action_server_ = rclcpp_action::create_server<SetTrackWidth>(
      this,
      "set_track_width",
      std::bind(&LegController::HandleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&LegController::HandleCancel, this, std::placeholders::_1),
      std::bind(&LegController::HandleAccepted, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&LegController::Tick, this));

    RCLCPP_INFO(get_logger(), "SetTrackWidth action + map(curvature) steering ready");
  }

private:
  void JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (i >= msg->position.size())
        break;
      current_positions_[msg->name[i]] = msg->position[i];
    }
    if (!initialized_ && current_positions_.size() >= 8) {
      initialized_ = true;
      RCLCPP_INFO(get_logger(), "joint_states ready");
    }
  }

  double JointAngleForMode(const JointDef & joint, SteeringMode mode) const
  {
    const double wide = get_parameter("wide_steering_angle").as_double();

    switch (mode) {
      case SteeringMode::kNarrowTrack:
        return 0.0;

      case SteeringMode::kWideTrack:
        if (joint.role == JointRole::kWheelArm) {
          return wide;
        }
        return 0.0;

      default:
        return 0.0;
    }
  }

  void BuildModeTargets(SteeringMode mode, std::unordered_map<std::string, double> & out) const
  {
    out.clear();
    for (const JointDef & joint : kJoints) {
      out[joint.name] = JointAngleForMode(joint, mode);
    }
  }

  void BuildMapTargets(
    SteeringMode mode,
    std::unordered_map<std::string, double> & out) const
  {
    const auto angles = my_robot_maps::map(steering_curvature_, ToTrackMode(mode));
    out.clear();
    for (size_t i = 0; i < kJoints.size(); ++i) {
      out[kJoints[i].name] = angles[i];
    }
  }

  void BuildSpinTargets(
    SteeringMode mode,
    std::unordered_map<std::string, double> & out) const
  {
    const auto angles = my_robot_maps::map_spin(
      steering_angle_deg_, ToSpinDirection(mode));
    out.clear();
    for (size_t i = 0; i < kJoints.size(); ++i) {
      out[kJoints[i].name] = angles[i];
    }
  }

  void BuildActionTargets(uint8_t target, std::unordered_map<std::string, double> & out) const
  {
    if (target == SetTrackWidth::Goal::TARGET_WIDE) {
      BuildModeTargets(SteeringMode::kWideTrack, out);
    } else {
      BuildModeTargets(SteeringMode::kNarrowTrack, out);
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
    if (initial_max_error_ < get_parameter("position_tolerance").as_double()) {
      initial_max_error_ = 1.0;
    }
  }

  void Tick()
  {
    if (!initialized_)
      return;

    if (active_goal_) {
      ExecuteAction();
      return;
    }
    PublishStableMode();
  }

  double ComputeMaxError(const std::unordered_map<std::string, double> & target) const
  {
    double max_err = 0.0;
    for (const auto & [name, goal_pos] : target) {
      const auto it = current_positions_.find(name);
      if (it == current_positions_.end())
        continue;
      max_err = std::max(max_err, std::abs(it->second - goal_pos));
    }
    return max_err;
  }

  void PublishJointCommand(const std::unordered_map<std::string, double> & positions)
  {
    sensor_msgs::msg::JointState cmd;
    cmd.header.stamp = now();
    for (const JointDef & joint : kJoints) {
      const auto it = positions.find(joint.name);
      if (it == positions.end())
        continue;
      cmd.name.push_back(joint.name);
      cmd.position.push_back(it->second);
    }
    if (!cmd.name.empty())
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

    for (auto & [name, goal_pos] : cmd) {
      const auto it = current_positions_.find(name);
      if (it == current_positions_.end())
        continue;
      cmd[name] = it->second + 0.15 * (goal_pos - it->second);
    }
    PublishJointCommand(cmd);

    const double max_err = ComputeMaxError(command_positions_);
    auto feedback = std::make_shared<SetTrackWidth::Feedback>();
    feedback->progress = static_cast<float>(
      std::clamp(1.0 - max_err / initial_max_error_, 0.0, 1.0));
    active_goal_->publish_feedback(feedback);

    if (max_err <= tolerance) {
      PublishJointCommand(command_positions_);
      auto result = std::make_shared<SetTrackWidth::Result>();
      result->success = true;
      result->message = "reached target";
      active_goal_->succeed(result);
      active_goal_.reset();
    }
  }

  void PublishStableMode()
  {
    SteeringMode mode = ModeFromString(mode_);
    if (mode == SteeringMode::kUnknown) {
      mode = SteeringMode::kNarrowTrack;
    }

    std::unordered_map<std::string, double> target;
    if (UsesCurvatureMap(mode)) {
      BuildMapTargets(mode, target);
    } else if (UsesSpinMap(mode)) {
      BuildSpinTargets(mode, target);
    } else {
      BuildModeTargets(mode, target);
    }
    PublishJointCommand(target);
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr curvature_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steering_wheel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Server<SetTrackWidth>::SharedPtr action_server_;

  std::unordered_map<std::string, double> current_positions_;
  std::unordered_map<std::string, double> command_positions_;
  std::shared_ptr<GoalHandleSetTrackWidth> active_goal_;
  uint8_t action_target_{0};
  double initial_max_error_{1.0};
  double steering_curvature_{0.0};
  double steering_angle_deg_{0.0};
  std::string mode_{"narrow_track"};
  bool initialized_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LegController>());
  rclcpp::shutdown();
  return 0;
}
