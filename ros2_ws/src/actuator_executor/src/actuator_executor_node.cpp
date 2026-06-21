// L2 执行器：SteeringCommand → joint_commands（唯一发布者，450ms 看门狗）

#include <array>
#include <chrono>
#include <memory>

#include "my_robot_msgs/msg/steering_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"

namespace {

struct JointDef {
  const char * name;
};

constexpr std::array<JointDef, 8> kJoints = {{
  {"link_002_joint"},
  {"link_003_joint"},
  {"link_005_joint"},
  {"link_006_joint"},
  {"link_008_joint"},
  {"link_009_joint"},
  {"link_011_joint"},
  {"link_012_joint"},
}};

}  // namespace

class ActuatorExecutorNode : public rclcpp::Node
{
public:
  ActuatorExecutorNode()
  : Node("actuator_executor")
  {
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

    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&ActuatorExecutorNode::Tick, this));

    RCLCPP_INFO(get_logger(), "actuator_executor: narrow_track joint_commands + watchdog");
  }

private:
  void JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    size_t count = 0;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (i < msg->position.size()) {
        ++count;
      }
    }
    if (!initialized_ && count >= 8) {
      initialized_ = true;
      std_msgs::msg::Bool ready;
      ready.data = true;
      ready_pub_->publish(ready);
      RCLCPP_INFO(get_logger(), "joint_states ready, system_ready=true");
    }
  }

  void Tick()
  {
    if (!initialized_) {
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
    if (!SteeringCommandStale() && last_steering_cmd_.valid) {
      for (size_t i = 0; i < positions.size(); ++i) {
        positions[i] = last_steering_cmd_.joint_positions[i];
      }
    }
    PublishJointCommand(positions);
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

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Subscription<my_robot_msgs::msg::SteeringCommand>::SharedPtr steering_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

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
