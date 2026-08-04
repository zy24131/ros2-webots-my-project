// L2 轮速控制：暂固定四轮速度为 0（差速逻辑待实现）

#include <chrono>
#include <memory>

#include "fsm/qos.hpp"
#include "fsm/topic_names.hpp"
#include "my_robot_msgs/msg/wheel_speeds.hpp"
#include "rclcpp/rclcpp.hpp"

class WheelSpeedControlNode : public rclcpp::Node
{
public:
  WheelSpeedControlNode()
  : Node("wheel_speed_control")
  {
    wheel_speeds_pub_ = create_publisher<my_robot_msgs::msg::WheelSpeeds>(
      fsm::topics::kWheelSpeeds,
      fsm::QoSFromParams(this, "qos.wheel_speeds_pub", "sensor_data"));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&WheelSpeedControlNode::Publish, this));

    RCLCPP_INFO(get_logger(), "wheel_speed_control: all wheel speeds fixed at 0");
  }

private:
  void Publish()
  {
    my_robot_msgs::msg::WheelSpeeds msg;
    msg.header.stamp = now();
    msg.valid = false;
    msg.speeds.fill(0.0);
    wheel_speeds_pub_->publish(msg);
  }

  rclcpp::Publisher<my_robot_msgs::msg::WheelSpeeds>::SharedPtr wheel_speeds_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelSpeedControlNode>());
  rclcpp::shutdown();
  return 0;
}
