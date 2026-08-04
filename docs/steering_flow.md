# 转向控制数据流

```text
/my_robot/steering_input (°)
        │
        ▼
steering_control ──► /my_robot/steering_command (SteeringCommand)
        │
        ▼
actuator_control ──► /my_robot/joint_commands  (唯一发布者，450ms 看门狗)
        ▲
        │ SetTrackAction（轮距切换）
fsm ──► /my_robot/mode

wheel_speed_control ──► /my_robot/internal/wheel_speeds
        ▼
hardware_bridge (Webots) ──► joint_states
```

## 节点职责

| 层 | 节点 | 职责 |
|----|------|------|
| L3 | `fsm` | Trigger 驱动 case 0~5，Action 客户端 |
| L2 | `steering_control` | map / map_spin → SteeringCommand |
| L2 | `actuator_control` | 合并命令 + Action + 看门狗 → joint_commands |
| L2 | `wheel_speed_control` | 四轮轮速（当前固定为 0） |
| L1 | `hardware_bridge` | Webots IO（真机替换 valve_driver） |

## 运动模式（SSOT）

FSM 状态、Service 意图、`/my_robot/mode` 字符串统一在 `fsm/motion.hpp`：

| 类型 | 用途 |
|------|------|
| `RobotMode` | FSM 状态 + mode 话题（case 0~5） |
| `Motion` | `set_motion` 服务（转向/自转） |
| `TrackIntent` | `set_track` 服务（轮距意图） |
| `FsmTrigger` | FSM 内部触发器 |

辅助函数：`ModeToString` / `ModeFromString`、`IsTrackDriving`、`IsSwitching`、`IsSpin`。

## 关节索引（SSOT）

命名与下标统一在 `fsm/include/fsm/joint_config.hpp`，各节点不再各自维护 `kJoints`。

| 索引体系 | 数量 | 用途 | 表名 |
|----------|------|------|------|
| `steering_index` | 8 | `SteeringCommand` / `joint_commands` | `kSteeringJoints` |
| `wheel_speed_index` | 4 | `WheelSpeeds.speeds` | `kDriveWheels` |
| `webots_index` | 12 | Webots 电机遍历 + `joint_states` | `kWebotsActuators` |

`kSteeringJoints` 顺序：节臂/轮臂交替（002 节、003 轮、005 节、006 轮 …）。  
自转只动轮臂，索引见 `kWheelArmIndices`（1,3,5,7）。





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
ros2 launch fsm my_robot.launch.xml



终端3 控制输入

source ~/ros2_webots/my_project/ros2_ws/install/setup.bash

# 转向 + 窄轮距
ros2 service call /my_robot/set_motion my_robot_msgs/srv/SetMotion "{mode: 0}"
ros2 service call /my_robot/set_track my_robot_msgs/srv/SetTrack "{track: 0}"

# 方向盘 30°（必须持续发，否则 450ms 后关节归零）
ros2 topic pub /my_robot/steering_input std_msgs/msg/Float64 "{data: 30.0}" -r 20


ros2 topic echo /my_robot/joint_commands --once
