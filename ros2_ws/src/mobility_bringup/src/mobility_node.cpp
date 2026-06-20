// L2 移动控制：按 FSM mode 发布轮速
//
// narrow_track / wide_track：发 wheel_speed（直行）
// switch_* / spin_*：发 0（切换轮距或自转时由 leg / 后续差速逻辑接管）

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

class MobilityNode : public rclcpp::Node
{
public:
  MobilityNode()
  : Node("mobility_node")
  {
    declare_parameter("wheel_speed", 0.2);
    mode_ = "narrow_track";
    pub_ = create_publisher<std_msgs::msg::Float64>("/my_robot/wheel_speed", 10);
    mode_sub_ = create_subscription<std_msgs::msg::String>(
      "/my_robot/mode", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) { mode_ = msg->data; });
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&MobilityNode::Publish, this));
    RCLCPP_INFO(get_logger(), "Mobility: wheel_speed on narrow_track / wide_track");
  }

private:
  void Publish()
  {
    std_msgs::msg::Float64 msg;
    const bool driving = (mode_ == "narrow_track" || mode_ == "wide_track");
    msg.data = driving ? get_parameter("wheel_speed").as_double() : 0.0;
    pub_->publish(msg);
  }

  std::string mode_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MobilityNode>());
  rclcpp::shutdown();
  return 0;
}
