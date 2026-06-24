# 转向控制数据流

```text
/my_robot/steering_wheel (°)
        │
        ▼
steering_controller ──► /my_robot/steering_command (SteeringCommand)
        │
        ▼
actuator_executor ──► /my_robot/joint_commands  (唯一发布者，450ms 看门狗)
        ▲
        │ SetTrackWidth Action（轮距切换）
robot_fsm ──► /my_robot/mode

/my_robot/internal/steering_curvature     ← 内部话题，不对 HMI 暴露
        ▼
mobility_controller ──► /my_robot/internal/wheel_speeds
        ▼
hardware_bridge (Webots) ──► joint_states
```

## 节点职责

| 层 | 节点 | 职责 |
|----|------|------|
| L3 | `robot_fsm` | Trigger 驱动 case 0~5，Action 客户端 |
| L2 | `steering_controller` | map / map_spin → SteeringCommand |
| L2 | `actuator_executor` | 合并命令 + Action + 看门狗 → joint_commands |
| L2 | `mobility_controller` | 曲率差速 WheelSpeeds |
| L1 | `hardware_bridge` | Webots IO（真机替换 valve_driver） |

## 就绪链

`actuator_executor` 收到 8 路 `joint_states` 后发布 `/my_robot/system_ready=true`。




## 编译
cd ~/ros2_webots/my_project/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

## 重新编译
cd ~/ros2_webots/my_project/ros2_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash


终端1 webots

source ~/ros2_webots/my_project/ros2_ws/install/setup.bash
cd ~/ros2_webots/my_project
webots --mode=realtime webots/worlds/my_project.wbt


终端2 控制栈

source ~/ros2_webots/my_project/ros2_ws/install/setup.bash
ros2 launch robot_fsm my_robot.launch.xml



终端3 控制输入

source ~/ros2_webots/my_project/ros2_ws/install/setup.bash

# 转向 + 窄轮距
ros2 service call /my_robot/set_motion_mode my_robot_msgs/srv/SetMotionMode "{mode: 0}"
ros2 service call /my_robot/set_track_width_switch my_robot_msgs/srv/SetTrackWidthSwitch "{track_width: 0}"

# 方向盘 30°（必须持续发，否则 450ms 后关节归零）
ros2 topic pub /my_robot/steering_wheel std_msgs/msg/Float64 "{data: 30.0}" -r 20


ros2 topic echo /my_robot/joint_commands --once
