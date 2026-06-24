// L2 移动控制：曲率差速 + 按 mode 启停
//
// narrow/wide_track：发布 WheelSpeeds（内外侧差速）
// switch_* / spin_*：valid=false，轮速 0

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "my_robot_msgs/msg/wheel_speeds.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

class MobilityControllerNode : public rclcpp::Node
{
public:
  MobilityControllerNode()
  : Node("mobility_controller")
  {
    declare_parameter("wheel_speed", 0.2);
    declare_parameter("track_half_width_narrow", 1.07);
    declare_parameter("track_half_width_wide", 1.25);

    wheel_speeds_pub_ = create_publisher<my_robot_msgs::msg::WheelSpeeds>(
      "/my_robot/internal/wheel_speeds", 10);
    legacy_speed_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/my_robot/internal/wheel_speed", 10);

    mode_sub_ = create_subscription<std_msgs::msg::String>(
      "/my_robot/mode", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) { mode_ = msg->data; });

    curvature_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/my_robot/internal/steering_curvature", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        curvature_ = msg->data;
      });

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&MobilityControllerNode::Publish, this));

    RCLCPP_INFO(get_logger(), "mobility_controller: curvature differential wheel speeds");
  }

private:
  double TrackHalfWidth() const
  {
    if (mode_ == "wide_track") {
      return get_parameter("track_half_width_wide").as_double();
    }
    return get_parameter("track_half_width_narrow").as_double();
  }

  void Publish()
  {
    const bool driving = (mode_ == "narrow_track" || mode_ == "wide_track");
    const double base_v = driving ? get_parameter("wheel_speed").as_double() : 0.0;

    my_robot_msgs::msg::WheelSpeeds speeds_msg;
    speeds_msg.header.stamp = now();
    speeds_msg.valid = driving;
    speeds_msg.speeds.fill(0.0);

    if (driving) {
      const double half_w = TrackHalfWidth();
      const double k = curvature_;
      // 右前/左前/右后/左后：右侧 +κW/2，左侧 -κW/2（相对 base_v 缩放）
      const double right = base_v * (1.0 + k * half_w);
      const double left = base_v * (1.0 - k * half_w);
      speeds_msg.speeds[0] = right;  // RF link_004
      speeds_msg.speeds[1] = left;   // LF link_007
      speeds_msg.speeds[2] = right;  // RR link_010
      speeds_msg.speeds[3] = left;   // LR link_013
    }

    wheel_speeds_pub_->publish(speeds_msg);

    std_msgs::msg::Float64 legacy;
    legacy.data = driving ? base_v : 0.0;
    legacy_speed_pub_->publish(legacy);
  }

  std::string mode_{"narrow_track"};
  double curvature_{0.0};
  rclcpp::Publisher<my_robot_msgs::msg::WheelSpeeds>::SharedPtr wheel_speeds_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr legacy_speed_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr curvature_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MobilityControllerNode>());
  rclcpp::shutdown();
  return 0;
}
