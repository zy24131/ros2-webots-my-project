// L1 硬件桥接（Webots 仿真实现；真机可替换 valve_driver/PLC 实现）
//
// 订阅：joint_commands、wheel_speeds（优先）/ wheel_speed（兼容）
// 发布：joint_states

#include <array>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "my_robot_msgs/msg/wheel_speeds.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Supervisor.hpp>

namespace {

struct JointInfo {
  const char * name;
  const char * sensor_name;
  bool is_wheel;
  int wheel_speed_index;  // WheelSpeeds[0..3]，非轮为 -1
};

constexpr JointInfo kJoints[] = {
  {"link_002_joint", "link_002_joint_sensor", false, -1},
  {"link_003_joint", "link_003_joint_sensor", false, -1},
  {"link_004_joint", nullptr, true, 0},
  {"link_005_joint", "link_005_joint_sensor", false, -1},
  {"link_006_joint", "link_006_joint_sensor", false, -1},
  {"link_007_joint", nullptr, true, 1},
  {"link_008_joint", "link_008_joint_sensor", false, -1},
  {"link_009_joint", "link_009_joint_sensor", false, -1},
  {"link_010_joint", nullptr, true, 2},
  {"link_011_joint", "link_011_joint_sensor", false, -1},
  {"link_012_joint", "link_012_joint_sensor", false, -1},
  {"link_013_joint", nullptr, true, 3},
};

}  // namespace

class HardwareBridgeNode : public rclcpp::Node
{
public:
  HardwareBridgeNode(webots::Supervisor * robot, int timestep)
  : Node("hardware_bridge"),
    robot_(robot),
    timestep_(timestep)
  {
    for (const JointInfo & joint : kJoints) {
      Actuator act;
      act.name = joint.name;
      act.motor = robot_->getMotor(joint.name);
      act.is_wheel = joint.is_wheel;
      act.wheel_speed_index = joint.wheel_speed_index;
      if (joint.sensor_name != nullptr) {
        act.sensor = robot_->getPositionSensor(joint.sensor_name);
        act.sensor->enable(timestep_);
      } else {
        act.sensor = nullptr;
        act.motor->setPosition(INFINITY);
      }
      actuators_.push_back(act);
      motor_by_name_[joint.name] = act.motor;
    }

    joint_cmd_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/my_robot/joint_commands", 10,
      std::bind(&HardwareBridgeNode::JointCommandCallback, this, std::placeholders::_1));

    wheel_speeds_sub_ = create_subscription<my_robot_msgs::msg::WheelSpeeds>(
      "/my_robot/wheel_speeds", 10,
      std::bind(&HardwareBridgeNode::WheelSpeedsCallback, this, std::placeholders::_1));

    wheel_speed_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/my_robot/wheel_speed", 10,
      std::bind(&HardwareBridgeNode::WheelSpeedCallback, this, std::placeholders::_1));

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "/my_robot/joint_states", 10);

    RCLCPP_INFO(get_logger(), "hardware_bridge (Webots): joint_commands + wheel_speeds IO");
  }

  void Step()
  {
    ApplyJointCommands();
    ApplyWheelSpeed();
    PublishJointStates();
  }

private:
  void JointCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    joint_commands_ = *msg;
    has_joint_commands_ = true;
  }

  void WheelSpeedsCallback(const my_robot_msgs::msg::WheelSpeeds::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    wheel_speeds_ = *msg;
    has_wheel_speeds_ = true;
  }

  void WheelSpeedCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    wheel_speed_ = msg->data;
  }

  void ApplyJointCommands()
  {
    sensor_msgs::msg::JointState cmd;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_joint_commands_) {
        return;
      }
      cmd = joint_commands_;
    }

    for (size_t i = 0; i < cmd.name.size(); ++i) {
      if (i >= cmd.position.size()) {
        continue;
      }
      const auto it = motor_by_name_.find(cmd.name[i]);
      if (it == motor_by_name_.end()) {
        continue;
      }
      for (const Actuator & act : actuators_) {
        if (act.motor != it->second) {
          continue;
        }
        if (!act.is_wheel) {
          it->second->setPosition(cmd.position[i]);
        }
        break;
      }
    }
  }

  void ApplyWheelSpeed()
  {
    my_robot_msgs::msg::WheelSpeeds speeds_msg;
    double legacy_speed = 0.0;
    bool use_speeds = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      legacy_speed = wheel_speed_;
      if (has_wheel_speeds_ && wheel_speeds_.valid) {
        speeds_msg = wheel_speeds_;
        use_speeds = true;
      }
    }

    for (Actuator & act : actuators_) {
      if (!act.is_wheel) {
        continue;
      }
      act.motor->setPosition(INFINITY);
      double speed = legacy_speed;
      if (use_speeds && act.wheel_speed_index >= 0 &&
        static_cast<size_t>(act.wheel_speed_index) < speeds_msg.speeds.size())
      {
        speed = speeds_msg.speeds[act.wheel_speed_index];
      }
      act.motor->setVelocity(speed);
    }
  }

  void PublishJointStates()
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = get_clock()->now();

    for (const Actuator & act : actuators_) {
      msg.name.push_back(act.name);
      if (act.sensor != nullptr) {
        const double pos = act.sensor->getValue();
        msg.position.push_back(std::isfinite(pos) ? pos : 0.0);
      } else {
        msg.position.push_back(0.0);
      }
      msg.velocity.push_back(act.motor->getVelocity());
      msg.effort.push_back(0.0);
    }
    joint_state_pub_->publish(msg);
  }

  struct Actuator {
    std::string name;
    webots::Motor * motor;
    webots::PositionSensor * sensor;
    bool is_wheel;
    int wheel_speed_index;
  };

  webots::Supervisor * robot_;
  int timestep_;
  std::vector<Actuator> actuators_;
  std::unordered_map<std::string, webots::Motor *> motor_by_name_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub_;
  rclcpp::Subscription<my_robot_msgs::msg::WheelSpeeds>::SharedPtr wheel_speeds_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr wheel_speed_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  std::mutex mutex_;
  sensor_msgs::msg::JointState joint_commands_;
  bool has_joint_commands_{false};
  my_robot_msgs::msg::WheelSpeeds wheel_speeds_;
  bool has_wheel_speeds_{false};
  double wheel_speed_{0.0};
};

int main(int argc, char ** argv)
{
  webots::Supervisor robot;
  const int timestep = static_cast<int>(robot.getBasicTimeStep());

  rclcpp::init(argc, argv);
  auto node = std::make_shared<HardwareBridgeNode>(&robot, timestep);

  std::thread spin_thread([node]() { rclcpp::spin(node); });

  while (robot.step(timestep) != -1) {
    node->Step();
  }

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
