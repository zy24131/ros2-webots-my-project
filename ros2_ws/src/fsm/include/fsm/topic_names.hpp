// ROS 话题 / Service / Action 名称 SSOT（Motion* / Track*）

#pragma once

namespace fsm {
namespace topics {

inline constexpr const char kMode[] = "/my_robot/mode";
inline constexpr const char kSteeringInput[] = "/my_robot/steering_input";
inline constexpr const char kSteeringCommand[] = "/my_robot/steering_command";
inline constexpr const char kJointCommands[] = "/my_robot/joint_commands";
inline constexpr const char kJointStates[] = "/my_robot/joint_states";
inline constexpr const char kImu[] = "/my_robot/imu";
inline constexpr const char kWheelSpeeds[] = "/my_robot/internal/wheel_speeds";

inline constexpr const char kSetMotion[] = "/my_robot/set_motion";
inline constexpr const char kSetTrack[] = "/my_robot/set_track";
inline constexpr const char kSetTrackAction[] = "set_track";

}  // namespace topics
}  // namespace fsm
/home/kemove/ros2_webots/my_project/ros2_ws/src/fsm/include/fsm/topic_names.hpp