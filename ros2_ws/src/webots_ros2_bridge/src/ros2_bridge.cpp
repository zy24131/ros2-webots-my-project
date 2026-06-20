// L1 硬件桥接：Webots Motor/Sensor ↔ ROS2 话题
// 只做 IO，不含锁腿、状态机等控制策略。
//
// 订阅：/my_robot/joint_commands（8 腿位置）、/my_robot/wheel_speed（4 轮 rad/s）
// 发布：/my_robot/joint_states（12 关节反馈）

#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Supervisor.hpp>

namespace {

struct JointInfo {
  const char * name;
  const char * sensor_name;  // 腿关节有 PositionSensor；轮胎为 nullptr
  bool is_wheel;
};

// 与 wbt 中 RotationalMotor / PositionSensor 名称一致
constexpr JointInfo kJoints[] = {
  {"link_002_joint", "link_002_joint_sensor", false},
  {"link_003_joint", "link_003_joint_sensor", false},
  {"link_004_joint", nullptr, true},
  {"link_005_joint", "link_005_joint_sensor", false},
  {"link_006_joint", "link_006_joint_sensor", false},
  {"link_007_joint", nullptr, true},
  {"link_008_joint", "link_008_joint_sensor", false},
  {"link_009_joint", "link_009_joint_sensor", false},
  {"link_010_joint", nullptr, true},
  {"link_011_joint", "link_011_joint_sensor", false},
  {"link_012_joint", "link_012_joint_sensor", false},
  {"link_013_joint", nullptr, true},
};

}  // namespace

class Ros2Bridge : public rclcpp::Node
{
public:
  Ros2Bridge(webots::Supervisor * robot, int timestep)
  : Node("ros2_webots_bridge"),
    robot_(robot),
    timestep_(timestep)
  {
    for (const JointInfo & joint : kJoints) {
      Actuator act;
      act.name = joint.name;
      act.motor = robot_->getMotor(joint.name);
      act.is_wheel = joint.is_wheel;
      if (joint.sensor_name != nullptr) {
        act.sensor = robot_->getPositionSensor(joint.sensor_name);
        act.sensor->enable(timestep_);
      } else {
        act.sensor = nullptr;
        // 速度模式：position=INFINITY 表示不跟踪角度，只跟 setVelocity
        act.motor->setPosition(INFINITY);
      }
      actuators_.push_back(act);
      motor_by_name_[joint.name] = act.motor;
    }

    joint_cmd_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/my_robot/joint_commands", 10,
      std::bind(&Ros2Bridge::JointCommandCallback, this, std::placeholders::_1));

    wheel_speed_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/my_robot/wheel_speed", 10,
      std::bind(&Ros2Bridge::WheelSpeedCallback, this, std::placeholders::_1));

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "/my_robot/joint_states", 10);

    RCLCPP_INFO(
      get_logger(),
      "IO bridge: joint_commands + wheel_speed in, joint_states out");
  }

  // 每个 Webots 仿真步调用一次（与 robot.step 同步）
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

  void WheelSpeedCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    wheel_speed_ = msg->data;
  }

  // 按关节名匹配，仅对非轮胎 motor 写 setPosition
  void ApplyJointCommands()
  {
    sensor_msgs::msg::JointState cmd;
    bool has_cmd = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_joint_commands_)
        return;
      cmd = joint_commands_;
      has_cmd = true;
    }
    if (!has_cmd)
      return;

    for (size_t i = 0; i < cmd.name.size(); ++i) {
      const auto it = motor_by_name_.find(cmd.name[i]);
      if (it == motor_by_name_.end())
        continue;
      if (i >= cmd.position.size())
        continue;

      for (const Actuator & act : actuators_) {
        if (act.motor != it->second)
          continue;
        if (act.is_wheel)
          break;
        it->second->setPosition(cmd.position[i]);
        break;
      }
    }
  }

  // 四轮同速；单位 rad/s，与 void 控制器一致
  void ApplyWheelSpeed()
  {
    double speed = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      speed = wheel_speed_;
    }
    for (Actuator & act : actuators_) {
      if (!act.is_wheel)
        continue;
      act.motor->setPosition(INFINITY);
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
        msg.position.push_back(0.0);  // 轮胎无角度传感器，占位
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
  };

  webots::Supervisor * robot_;
  int timestep_;
  std::vector<Actuator> actuators_;
  std::unordered_map<std::string, webots::Motor *> motor_by_name_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr wheel_speed_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  std::mutex mutex_;  // 保护回调线程与 Step 线程之间的命令缓存
  sensor_msgs::msg::JointState joint_commands_;
  bool has_joint_commands_{false};
  double wheel_speed_{0.0};
};

int main(int argc, char ** argv)
{
  webots::Supervisor robot;
  const int timestep = static_cast<int>(robot.getBasicTimeStep());

  rclcpp::init(argc, argv);
  auto node = std::make_shared<Ros2Bridge>(&robot, timestep);

  // ROS 回调在独立线程；Webots step 在主线程
  std::thread spin_thread([node]() { rclcpp::spin(node); });

  while (robot.step(timestep) != -1)
    node->Step();

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
