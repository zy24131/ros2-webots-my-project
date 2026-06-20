// L2 转向控制：方向盘 + mode → SteeringCommand + steering_curvature

#include <array>
#include <chrono>
#include <memory>
#include <string>

#include "my_robot_maps/curvature_to_joints.hpp"
#include "my_robot_maps/spin_to_joints.hpp"
#include "my_robot_maps/steering_angle_to_curvature.hpp"
#include "my_robot_maps/types.hpp"
#include "my_robot_msgs/msg/steering_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

namespace {

enum class DriveMode {
  kNarrowTrack,
  kWideTrack,
  kSpinLeft,
  kSpinRight,
  kSwitching,
  kUnknown,
};

DriveMode ModeFromString(const std::string & mode)
{
  if (mode == "narrow_track") {
    return DriveMode::kNarrowTrack;
  }
  if (mode == "wide_track") {
    return DriveMode::kWideTrack;
  }
  if (mode == "spin_left") {
    return DriveMode::kSpinLeft;
  }
  if (mode == "spin_right") {
    return DriveMode::kSpinRight;
  }
  if (mode == "switch_to_wide" || mode == "switch_to_narrow") {
    return DriveMode::kSwitching;
  }
  return DriveMode::kUnknown;
}

my_robot_maps::TrackMode ToTrackMode(DriveMode mode)
{
  return mode == DriveMode::kWideTrack ?
         my_robot_maps::TrackMode::kWide :
         my_robot_maps::TrackMode::kNarrow;
}

my_robot_maps::TrackMode TrackModeFromFsmMode(const std::string & mode)
{
  if (mode == "wide_track" || mode == "switch_to_wide") {
    return my_robot_maps::TrackMode::kWide;
  }
  return my_robot_maps::TrackMode::kNarrow;
}

}  // namespace

class SteeringControllerNode : public rclcpp::Node
{
public:
  SteeringControllerNode()
  : Node("steering_controller")
  {
    wheel_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/my_robot/steering_wheel", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        steering_angle_deg_ = msg->data;
      });

    mode_sub_ = create_subscription<std_msgs::msg::String>(
      "/my_robot/mode", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        mode_ = msg->data;
      });

    cmd_pub_ = create_publisher<my_robot_msgs::msg::SteeringCommand>(
      "/my_robot/steering_command", 10);
    curvature_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/my_robot/steering_curvature", 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&SteeringControllerNode::Publish, this));

    RCLCPP_INFO(get_logger(), "steering_wheel -> steering_command + curvature");
  }

private:
  void Publish()
  {
    const DriveMode drive_mode = ModeFromString(mode_);
    const my_robot_maps::TrackMode track_mode = TrackModeFromFsmMode(mode_);

    std_msgs::msg::Float64 curvature_msg;
    curvature_msg.data = my_robot_maps::steering_angle_to_curvature(
      steering_angle_deg_, track_mode);
    curvature_pub_->publish(curvature_msg);

    my_robot_msgs::msg::SteeringCommand cmd;
    cmd.header.stamp = now();
    cmd.valid = false;
    cmd.source = my_robot_msgs::msg::SteeringCommand::SOURCE_IDLE;
    cmd.joint_positions.fill(0.0);

    if (drive_mode == DriveMode::kSwitching) {
      cmd_pub_->publish(cmd);
      return;
    }

    if (drive_mode == DriveMode::kSpinLeft || drive_mode == DriveMode::kSpinRight) {
      const auto direction = drive_mode == DriveMode::kSpinLeft ?
        my_robot_maps::SpinDirection::kLeft :
        my_robot_maps::SpinDirection::kRight;
      const auto angles = my_robot_maps::map_spin(steering_angle_deg_, direction);
      FillCommand(cmd, angles, my_robot_msgs::msg::SteeringCommand::SOURCE_SPIN);
      cmd_pub_->publish(cmd);
      return;
    }

    DriveMode map_mode = drive_mode;
    if (map_mode == DriveMode::kUnknown) {
      map_mode = DriveMode::kNarrowTrack;
    }

    const double curvature = my_robot_maps::steering_angle_to_curvature(
      steering_angle_deg_, ToTrackMode(map_mode));
    const auto angles = my_robot_maps::map(curvature, ToTrackMode(map_mode));
    FillCommand(cmd, angles, my_robot_msgs::msg::SteeringCommand::SOURCE_MAP);
    cmd_pub_->publish(cmd);
  }

  template<typename JointArray>
  void FillCommand(
    my_robot_msgs::msg::SteeringCommand & cmd,
    const JointArray & angles,
    uint8_t source)
  {
    cmd.valid = true;
    cmd.source = source;
    for (size_t i = 0; i < angles.size() && i < cmd.joint_positions.size(); ++i) {
      cmd.joint_positions[i] = angles[i];
    }
  }

  double steering_angle_deg_{0.0};
  std::string mode_{"narrow_track"};
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr wheel_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Publisher<my_robot_msgs::msg::SteeringCommand>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr curvature_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SteeringControllerNode>());
  rclcpp::shutdown();
  return 0;
}
