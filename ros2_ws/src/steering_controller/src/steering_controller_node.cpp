// L2 转向控制：窄轮距方向盘 → SteeringCommand

#include <array>
#include <chrono>
#include <memory>

#include "my_robot_maps/curvature_to_joints.hpp"
#include "my_robot_maps/steering_angle_to_curvature.hpp"
#include "my_robot_maps/types.hpp"
#include "my_robot_msgs/msg/steering_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace {

constexpr my_robot_maps::TrackMode kTrackMode = my_robot_maps::TrackMode::kNarrow;

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

    cmd_pub_ = create_publisher<my_robot_msgs::msg::SteeringCommand>(
      "/my_robot/steering_command", 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&SteeringControllerNode::Publish, this));

    RCLCPP_INFO(get_logger(), "narrow_track: steering_wheel -> steering_command");
  }

private:
  void Publish()
  {
    const double curvature = my_robot_maps::steering_angle_to_curvature(
      steering_angle_deg_, kTrackMode);
    const auto angles = my_robot_maps::map(curvature, kTrackMode);

    my_robot_msgs::msg::SteeringCommand cmd;
    cmd.header.stamp = now();
    cmd.valid = true;
    cmd.source = my_robot_msgs::msg::SteeringCommand::SOURCE_MAP;
    for (size_t i = 0; i < angles.size() && i < cmd.joint_positions.size(); ++i) {
      cmd.joint_positions[i] = angles[i];
    }
    cmd_pub_->publish(cmd);
  }

  double steering_angle_deg_{0.0};
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr wheel_sub_;
  rclcpp::Publisher<my_robot_msgs::msg::SteeringCommand>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SteeringControllerNode>());
  rclcpp::shutdown();
  return 0;
}
