// L2 移动控制：窄轮距固定轮速，四轮同速

#include <chrono>
#include <memory>

#include "my_robot_msgs/msg/wheel_speeds.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class MobilityControllerNode : public rclcpp::Node
{
public:
  MobilityControllerNode()
  : Node("mobility_controller")
  {
    declare_parameter("wheel_speed", 0.2);

    wheel_speeds_pub_ = create_publisher<my_robot_msgs::msg::WheelSpeeds>(
      "/my_robot/wheel_speeds", 10);
    legacy_speed_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/my_robot/wheel_speed", 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&MobilityControllerNode::Publish, this));

    RCLCPP_INFO(get_logger(), "narrow_track: uniform wheel_speed");
  }

private:
  void Publish()
  {
    const double v = get_parameter("wheel_speed").as_double();

    my_robot_msgs::msg::WheelSpeeds speeds_msg;
    speeds_msg.header.stamp = now();
    speeds_msg.valid = true;
    speeds_msg.speeds.fill(v);
    wheel_speeds_pub_->publish(speeds_msg);

    std_msgs::msg::Float64 legacy;
    legacy.data = v;
    legacy_speed_pub_->publish(legacy);
  }

  rclcpp::Publisher<my_robot_msgs::msg::WheelSpeeds>::SharedPtr wheel_speeds_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr legacy_speed_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MobilityControllerNode>());
  rclcpp::shutdown();
  return 0;
}
