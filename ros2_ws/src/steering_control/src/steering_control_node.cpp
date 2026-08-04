// L2 转向控制：方向盘 + mode → SteeringCommand

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "fsm/motion.hpp"
#include "fsm/qos.hpp"
#include "fsm/topic_names.hpp"
#include "robot_kinematics/curvature_to_joints.hpp"
#include "robot_kinematics/spin_to_joints.hpp"
#include "robot_kinematics/steering_angle_to_curvature.hpp"
#include "robot_kinematics/types.hpp"
#include "my_robot_msgs/msg/steering_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

class SteeringControllerNode : public rclcpp::Node
{
public:
  /**
   * @brief 转向控制器节点构造函数
   * 
   * 初始化转向控制器节点，完成以下功能：
   * - 声明控制器参数：宽轮距中立角度(度)、方向盘死区(度)
   * - 订阅方向盘角度话题（20ms周期）
   * - 订阅机器人模式话题
   * - 创建转向命令发布者
   * - 启动20ms周期的定时器，定期发布转向命令
   */
  SteeringControllerNode()
  : Node("steering_control")
  {
    // 声明控制器参数：宽轮距中立角度(度)、方向盘死区(度)
    declare_parameter("wide_neutral_angle_deg", 45.0);
    declare_parameter("steering_deadband_deg", 3.0);

    // 订阅方向盘角度，20ms周期发布转向命令

    wheel_sub_ = create_subscription<std_msgs::msg::Float64>(
      fsm::topics::kSteeringInput,
      fsm::QoSFromParams(this, "qos.steering_input_sub", "sensor_data"),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        steering_angle_deg_ = msg->data;
      });

    mode_sub_ = create_subscription<std_msgs::msg::String>(
      fsm::topics::kMode,
      fsm::QoSFromParams(this, "qos.mode_sub", "reliable"),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        mode_ = fsm::ModeFromString(msg->data);
      });

    cmd_pub_ = create_publisher<my_robot_msgs::msg::SteeringCommand>(
      fsm::topics::kSteeringCommand,
      fsm::QoSFromParams(this, "qos.steering_command_pub", "reliable"));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&SteeringControllerNode::Publish, this));

    RCLCPP_INFO(get_logger(), "steering_input -> steering_command");
  }

private:
  /**
   * @brief 定时发布回调函数
   * 
   * 20ms周期执行，根据当前模式和方向盘角度计算关节目标角度并发布：
   * - 模式切换期间停止输出
   * - 原地旋转模式：使用 spin map 计算转向角度
   * - 广角模式死区内：回到中立姿态（所有轮偏转45度）
   * - 正常转向模式：根据方向盘角度和轨距模式计算转向关节位置
   */
  void Publish()
  {
    fsm::RobotMode drive_mode = mode_;

    my_robot_msgs::msg::SteeringCommand cmd;
    cmd.header.stamp = now();
    cmd.valid = false;
    cmd.source = my_robot_msgs::msg::SteeringCommand::SOURCE_IDLE;
    cmd.joint_positions.fill(0.0);

    // 切换模式时停止输出
    if (fsm::IsSwitching(drive_mode)) {
      cmd_pub_->publish(cmd);
      return;
    }

    // 原地旋转模式：使用 spin map 计算转向角度
    if (fsm::IsSpin(drive_mode)) {
      const auto direction = drive_mode == fsm::kSpinLeft ?
        robot_kinematics::SpinDirection::kLeft :
        robot_kinematics::SpinDirection::kRight;
      const auto angles = robot_kinematics::map_spin(0.0, direction);
      FillCommand(cmd, angles, my_robot_msgs::msg::SteeringCommand::SOURCE_SPIN);
      cmd_pub_->publish(cmd);
      return;
    }

    // 未知模式默认窄轨
    if (drive_mode == fsm::kRobotModeUnknown) {
      drive_mode = fsm::kNarrowTrack;
    }

    const robot_kinematics::TrackMode track_mode =
      drive_mode == fsm::kWideTrack ?
      robot_kinematics::TrackMode::kWide :
      robot_kinematics::TrackMode::kNarrow;
    const double deadband = get_parameter("steering_deadband_deg").as_double();

    // 宽轮距模式死区内：回到中立姿态（所有轮偏转45度）
    if (drive_mode == fsm::kWideTrack &&
      std::abs(steering_angle_deg_) < deadband)
    {
      const double neutral_rad =
        get_parameter("wide_neutral_angle_deg").as_double() * M_PI / 180.0;
      robot_kinematics::JointAngles angles{};
      angles.fill(neutral_rad);

      FillCommand(cmd, angles, my_robot_msgs::msg::SteeringCommand::SOURCE_MAP);
      cmd_pub_->publish(cmd);
      return;
    }

    // 正常转向：方向盘角度 → 曲率 → 关节角度
    const double curvature = robot_kinematics::steering_angle_to_curvature(
      steering_angle_deg_, track_mode);

    const auto angles = robot_kinematics::map(curvature, track_mode);
    FillCommand(cmd, angles, my_robot_msgs::msg::SteeringCommand::SOURCE_MAP);
    cmd_pub_->publish(cmd);
  }

  // 辅助函数：将关节角度填充到 SteeringCommand 消息
  template<typename JointArray>
  void FillCommand(
    my_robot_msgs::msg::SteeringCommand & cmd,
    const JointArray & angles,
    uint8_t source)
  {
    cmd.valid = true;
    cmd.source = source;
    for (size_t i = 0; i < angles.size() && i < cmd.joint_positions.size(); ++i) {
      cmd.joint_positions[i] = angles[i];
    }
  }

  double steering_angle_deg_{0.0};
  fsm::RobotMode mode_{fsm::kNarrowTrack};
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr wheel_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
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
