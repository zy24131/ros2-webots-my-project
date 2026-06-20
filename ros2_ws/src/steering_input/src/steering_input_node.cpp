// 方向盘输入 (°) → 曲率，按 FSM 轮距模式选用窄/宽 al.cpp 算法

#include <chrono>
#include <memory>
#include <string>

#include "my_robot_maps/steering_angle_to_curvature.hpp"
#include "my_robot_maps/types.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

namespace {

my_robot_maps::TrackMode TrackModeFromFsmMode(const std::string & mode)
{
  if (mode == "wide_track" || mode == "switch_to_wide") {
    return my_robot_maps::TrackMode::kWide;
  }
  return my_robot_maps::TrackMode::kNarrow;
}

}  // namespace

class SteeringInputNode : public rclcpp::Node
{
public:
  SteeringInputNode()
  : Node("steering_input_node")
  {
    wheel_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/my_robot/steering_wheel", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        steering_angle_deg_ = msg->data;
      });

    mode_sub_ = create_subscription<std_msgs::msg::String>(
      "/my_robot/mode", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        track_mode_ = TrackModeFromFsmMode(msg->data);
      });

    curvature_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/my_robot/steering_curvature", 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&SteeringInputNode::Publish, this));

    RCLCPP_INFO(get_logger(), "steering_wheel (deg) -> steering_curvature via al.cpp maps");
  }

private:
  void Publish()
  {
    std_msgs::msg::Float64 msg;
    msg.data = my_robot_maps::steering_angle_to_curvature(
      steering_angle_deg_, track_mode_);
    curvature_pub_->publish(msg);
  }

  double steering_angle_deg_{0.0};
  my_robot_maps::TrackMode track_mode_{my_robot_maps::TrackMode::kNarrow};
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr wheel_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr curvature_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SteeringInputNode>());
  rclcpp::shutdown();
  return 0;
}
