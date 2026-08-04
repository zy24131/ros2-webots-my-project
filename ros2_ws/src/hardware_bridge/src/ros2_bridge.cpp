// L1 硬件桥接（Webots 仿真实现；真机可替换 valve_driver/PLC 实现）
//
// 订阅：joint_commands、internal/wheel_speeds
// 发布：joint_states、imu

#include <array>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "fsm/joint_config.hpp"
#include "fsm/qos.hpp"
#include "fsm/topic_names.hpp"
#include "my_robot_msgs/msg/wheel_speeds.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Supervisor.hpp>

namespace {

struct Actuator {
  std::string name;
  webots::Motor * motor;
  webots::PositionSensor * sensor;
  bool is_drive_wheel;
  int wheel_speed_index;
};

void RpyToQuaternion(
  double roll, double pitch, double yaw,
  double & qx, double & qy, double & qz, double & qw)
{
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  qw = cr * cp * cy + sr * sp * sy;
  qx = sr * cp * cy - cr * sp * sy;
  qy = cr * sp * cy + sr * cp * sy;
  qz = cr * cp * sy - sr * sp * cy;
}

}  // namespace

class HardwareBridgeNode : public rclcpp::Node
{
public:
  HardwareBridgeNode(webots::Supervisor * robot, int timestep)
  : Node("hardware_bridge"),
    robot_(robot),
    timestep_(timestep)
  {
    for (const fsm::WebotsActuatorSpec & spec : fsm::kWebotsActuators) {
      Actuator act;
      act.name = spec.motor_name;
      act.motor = robot_->getMotor(spec.motor_name);
      act.is_drive_wheel = spec.is_drive_wheel;
      act.wheel_speed_index = spec.wheel_speed_index;
      if (spec.sensor_name != nullptr) {
        act.sensor = robot_->getPositionSensor(spec.sensor_name);
        act.sensor->enable(timestep_);
      } else {
        act.sensor = nullptr;
        act.motor->setPosition(INFINITY);
      }
      actuators_.push_back(act);
      motor_by_name_[spec.motor_name] = act.motor;
    }

    joint_cmd_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      fsm::topics::kJointCommands,
      fsm::QoSFromParams(this, "qos.joint_commands_sub", "reliable"),
      std::bind(&HardwareBridgeNode::JointCommandCallback, this, std::placeholders::_1));

    wheel_speeds_sub_ = create_subscription<my_robot_msgs::msg::WheelSpeeds>(
      fsm::topics::kWheelSpeeds,
      fsm::QoSFromParams(this, "qos.wheel_speeds_sub", "sensor_data"),
      std::bind(&HardwareBridgeNode::WheelSpeedsCallback, this, std::placeholders::_1));

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      fsm::topics::kJointStates,
      fsm::QoSFromParams(this, "qos.joint_states_pub", "sensor_data"));

    imu_accel_ = robot_->getAccelerometer("imu_accelerometer");
    imu_gyro_ = robot_->getGyro("imu_gyro");
    imu_inertial_ = robot_->getInertialUnit("imu_inertial_unit");
    if (imu_accel_ && imu_gyro_ && imu_inertial_) {
      imu_accel_->enable(timestep_);
      imu_gyro_->enable(timestep_);
      imu_inertial_->enable(timestep_);
      imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
        fsm::topics::kImu,
        fsm::QoSFromParams(this, "qos.imu_pub", "sensor_data"));
      has_imu_ = true;
      RCLCPP_INFO(get_logger(), "IMU enabled at model center -> %s", fsm::topics::kImu);
    } else {
      RCLCPP_WARN(get_logger(), "IMU devices not found in Webots model");
    }

    RCLCPP_INFO(get_logger(), "hardware_bridge (Webots): joint_commands + internal/wheel_speeds IO");
  }

  void Step()
  {
    ApplyJointCommands();
    ApplyWheelSpeed();
    PublishJointStates();
    PublishImu();
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

  void ApplyWheelSpeed()
  {
    my_robot_msgs::msg::WheelSpeeds speeds_msg;
    bool has_speeds = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (has_wheel_speeds_) {
        speeds_msg = wheel_speeds_;
        has_speeds = true;
      }
    }

    for (Actuator & act : actuators_) {
      if (!act.is_drive_wheel) {
        continue;
      }
      act.motor->setPosition(INFINITY);
      double speed = 0.0;
      if (has_speeds && speeds_msg.valid && act.wheel_speed_index >= 0 &&
        static_cast<size_t>(act.wheel_speed_index) < speeds_msg.speeds.size())
      {
        speed = speeds_msg.speeds[act.wheel_speed_index];
      }
      act.motor->setVelocity(speed);
    }
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
        if (!act.is_drive_wheel) {
          it->second->setPosition(cmd.position[i]);
        }
        break;
      }
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

  void PublishImu()
  {
    if (!has_imu_ || !imu_pub_) {
      return;
    }

    const double * acc = imu_accel_->getValues();
    const double * gyro = imu_gyro_->getValues();
    const double * rpy = imu_inertial_->getRollPitchYaw();

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = "imu_link";
    msg.linear_acceleration.x = acc[0];
    msg.linear_acceleration.y = acc[1];
    msg.linear_acceleration.z = acc[2];
    msg.angular_velocity.x = gyro[0];
    msg.angular_velocity.y = gyro[1];
    msg.angular_velocity.z = gyro[2];
    RpyToQuaternion(
      rpy[0], rpy[1], rpy[2],
      msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
    msg.orientation_covariance[0] = -1.0;
    imu_pub_->publish(msg);
  }

  webots::Supervisor * robot_;
  int timestep_;
  std::vector<Actuator> actuators_;
  std::unordered_map<std::string, webots::Motor *> motor_by_name_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub_;
  rclcpp::Subscription<my_robot_msgs::msg::WheelSpeeds>::SharedPtr wheel_speeds_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  webots::Accelerometer * imu_accel_{nullptr};
  webots::Gyro * imu_gyro_{nullptr};
  webots::InertialUnit * imu_inertial_{nullptr};
  bool has_imu_{false};

  std::mutex mutex_;
  sensor_msgs::msg::JointState joint_commands_;
  bool has_joint_commands_{false};
  my_robot_msgs::msg::WheelSpeeds wheel_speeds_;
  bool has_wheel_speeds_{false};
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
