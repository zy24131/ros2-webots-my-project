# my_project — ROS2 + Webots

```text
my_project/
├── webots/                     # 仿真模型
├── ros2_ws/src/
│   ├── my_robot_msgs/          # Msg / Srv / Action
│   ├── robot_kinematics/          # 映射算法库（纯 C++）
│   ├── hardware_bridge/        # L1：IO 桥接（Webots；真机可换驱动）
│   ├── steering_control/    # L2：方向盘 → SteeringCommand
│   ├── actuator_control/      # L2：唯一 joint_commands 发布者
│   ├── wheel_speed_control/    # L2：曲率差速轮速
│   └── fsm/                    # L3 状态机 + 公共头文件（motion_mode/joint_config/QoS）+ launch/config
├── run_webots.sh
└── docs/
```

## 编译与运行

```bash
cd ~/ros2_webots/my_project/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select my_robot_msgs robot_kinematics hardware_bridge steering_control actuator_control wheel_speed_control fsm
source install/setup.bash

# 终端1：Webots + hardware_bridge
cd ~/ros2_webots/my_project && bash run_webots.sh

# 终端2：控制栈
ros2 launch fsm my_robot.launch.xml
```

## 控制数据流

详见 [docs/steering_flow.md](docs/steering_flow.md)。  
ROS2 通信机制说明见 [docs/ros2_communication.md](docs/ros2_communication.md)。

```text
steering_input → steering_control → steering_command
                                           ↓
                                    actuator_control → joint_commands
steering_curvature → wheel_speed_control → wheel_speeds
```

## 映射算法库 `robot_kinematics`

窄/宽轮距链（al.cpp）+ 自转 `map_spin()`。算法输出度，`map()` 内 deg2rad。

## FSM 状态（case 0~5）

| case | mode | 说明 |
|------|------|------|
| 0 | `narrow_track` | 窄轮距转向 |
| 1 | `switch_to_wide` | Action 切宽中 |
| 2 | `wide_track` | 宽轮距转向 |
| 3 | `switch_to_narrow` | Action 切窄中 |
| 4 | `spin_left` | 原地左转 |
| 5 | `spin_right` | 原地右转 |

## 主要话题

| 话题 | 说明 |
|------|------|
| `/my_robot/steering_input` | 方向盘输入，有符号转向角（度） |
| `/my_robot/steering_command` | 8 关节目标（SteeringCommand） |
| `/my_robot/steering_curvature` | 曲率 κ |
| `/my_robot/joint_commands` | 8 腿最终命令（actuator_control 唯一发布） |
| `/my_robot/wheel_speeds` | 四轮差速 rad/s |
| `/my_robot/mode` | FSM 当前 mode |
| `/my_robot/set_motion_mode` | 运动模式 Service |
| `/my_robot/set_track_width_switch` | 轮距意图 Service |
| `/set_track_width` | 轮距切换 Action |

模式切换示例：

```bash
# 转向模式
ros2 service call /my_robot/set_motion_mode my_robot_msgs/srv/SetMotionMode "{mode: 0}"
# 逆时针自转
ros2 service call /my_robot/set_motion_mode my_robot_msgs/srv/SetMotionMode "{mode: 2}"
# 切宽轮距意图
ros2 service call /my_robot/set_track_width_switch my_robot_msgs/srv/SetTrackWidthSwitch "{track_width: 1}"
```

## Launch 分层

| 文件 | 内容 |
|------|------|
| `control_bringup.launch.xml` | 状态机 + 转向控制 + 执行器 + 轮速控制 |
| `my_robot.launch.xml` | include control_bringup |
| `sim_bringup.launch.xml` | 仿真说明（Webots 由 run_webots.sh 启动） |

参数：`fsm/config/my_robot.yaml`

## 修改指南

| 目标 | 文件 |
|------|------|
| 转向映射 | `robot_kinematics/src/` |
| SteeringCommand 生成 | `steering_control/` |
| 命令合并/看门狗/Action | `actuator_control/` |
| 轮速差速 | `wheel_speed_control/`（当前固定 0，待实现） |
| FSM | `fsm/src/fsm_node.cpp` |
| Webots/真机 IO | `hardware_bridge/` |
