# my_project — ROS2 + Webots

```text
my_project/
├── webots/                     # 仿真模型
├── ros2_ws/src/
│   ├── my_robot_msgs/          # SteeringCommand, WheelSpeeds, SetTrackWidth
│   ├── my_robot_maps/          # 映射算法库（纯 C++）
│   ├── hardware_bridge/        # L1：IO 桥接（Webots；真机可换驱动）
│   ├── steering_controller/    # L2：方向盘 → SteeringCommand
│   ├── actuator_executor/      # L2：唯一 joint_commands 发布者
│   ├── mobility_controller/    # L2：曲率差速轮速
│   └── robot_fsm/              # L3：Trigger FSM + launch/config
├── run_webots.sh
└── docs/
```

## 编译与运行

```bash
cd ~/ros2_webots/my_project/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select my_robot_msgs my_robot_maps hardware_bridge steering_controller actuator_executor mobility_controller robot_fsm
source install/setup.bash

# 终端1：Webots + hardware_bridge
cd ~/ros2_webots/my_project && bash run_webots.sh

# 终端2：控制栈
ros2 launch robot_fsm my_robot.launch.xml
```

## 控制数据流

详见 [docs/steering_flow.md](docs/steering_flow.md)。  
ROS2 通信机制说明见 [docs/ros2_communication.md](docs/ros2_communication.md)。

```text
steering_wheel → steering_controller → steering_command
                                           ↓
                                    actuator_executor → joint_commands
steering_curvature → mobility_controller → wheel_speeds
```

## 映射算法库 `my_robot_maps`

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
| `/my_robot/steering_wheel` | 有符号转向角（度） |
| `/my_robot/steering_command` | 8 关节目标（SteeringCommand） |
| `/my_robot/steering_curvature` | 曲率 κ |
| `/my_robot/joint_commands` | 8 腿最终命令（actuator_executor 唯一发布） |
| `/my_robot/wheel_speeds` | 四轮差速 rad/s |
| `/my_robot/system_ready` | joint_states 就绪 |
| `/my_robot/mode` | FSM 当前 mode |
| `/set_track_width` | 轮距切换 Action |

## Launch 分层

| 文件 | 内容 |
|------|------|
| `control_bringup.launch.xml` | fsm + steering + actuator + mobility |
| `my_robot.launch.xml` | include control_bringup |
| `sim_bringup.launch.xml` | 仿真说明（Webots 由 run_webots.sh 启动） |

参数：`robot_fsm/config/my_robot.yaml`

## 修改指南

| 目标 | 文件 |
|------|------|
| 转向映射 | `my_robot_maps/src/` |
| SteeringCommand 生成 | `steering_controller/` |
| 命令合并/看门狗/Action | `actuator_executor/` |
| 轮速差速 | `mobility_controller/` |
| FSM | `robot_fsm/src/robot_fsm.cpp` |
| Webots/真机 IO | `hardware_bridge/` |
