# my_project — ROS2 + Webots

与 `ros_bridge` 相同布局：**Webots 模型** 与 **ros2_ws** 是两个并列文件夹。

```text
my_project/
├── webots/                     # 仿真模型
├── ros2_ws/src/
│   ├── my_robot_msgs/          # SetTrackWidth.action
│   ├── my_robot_maps/          # 映射算法库（纯 C++，无 ROS 依赖）
│   ├── webots_ros2_bridge/     # L1：IO 桥接
│   ├── steering_input/         # L2：方向盘 → 曲率
│   ├── mobility_bringup/       # L2：轮速
│   ├── leg_controller/         # L2：map(曲率) → 8 腿 + Action
│   └── robot_fsm/              # L3：case 0~5 状态机
├── run_webots.sh
└── docs/
```

## 编译与运行

```bash
cd ~/ros2_webots/my_project/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select my_robot_msgs my_robot_maps webots_ros2_bridge mobility_bringup steering_input leg_controller robot_fsm
source install/setup.bash

# 终端1
cd ~/ros2_webots/my_project && bash run_webots.sh

# 终端2
ros2 launch robot_fsm my_robot.launch.xml
```

## 方向盘 → 曲率 → 8 腿（两步）

```text
/my_robot/steering_wheel (°)  →  steering_input  →  /my_robot/steering_curvature
        ↑ 订阅 /my_robot/mode (窄/宽)                    ↓
                                              leg_controller::map()
                                                      ↓
                                              /my_robot/joint_commands
```

| 步骤 | 节点 | 公式 / 接口 |
|------|------|-------------|
| 1 | `steering_input` | `steering_angle_to_curvature(angle_deg, track_mode)` |
| 2 | `leg_controller` | `map(curvature, track_mode)` → 8×rad |

## 映射算法库 `my_robot_maps`

纯 C++ 库（源自 `al.cpp`），集中放置窄/宽轮距映射链；节点只负责 ROS 话题收发。

```text
ros2_ws/src/my_robot_maps/
├── include/my_robot_maps/
│   ├── types.hpp
│   ├── narrow_track_maps.hpp       # 窄轮距 4 链
│   ├── wide_track_maps.hpp         # 宽轮距 4 链
│   ├── steering_angle_to_curvature.hpp
│   ├── curvature_to_joints.hpp     # map(κ, mode)
│   └── spin_to_joints.hpp          # map_spin(°, direction)
└── src/
    ├── narrow_track_maps.cpp
    ├── wide_track_maps.cpp
    ├── steering_angle_to_curvature.cpp
    ├── curvature_to_joints.cpp
    └── spin_to_joints.cpp
```

**窄轮距**：SteeringAngle(°) → Curvature → Wheel_Out → Middle → Arm_In  
节臂(1,3,5,7)←Arm_In，轮臂(2,4,6,8)←Wheel_Out；C_MAX=1/2.14，|角|>80° 限 80，|角|<3°→0。

**宽轮距**：SteeringAngle(°) → Curvature → Arm_Out → Middle → Wheel_In  
节臂←Arm_Out，轮臂←Wheel_In；C_MAX=1/2.5，|角|<3°→0。

输入为有符号度数（如 −30° / +30°）；链内用绝对值，曲率与关节角乘 sign。算法输出为度，`map()` 内 `deg2rad()` 后写入 Webots。

**自转** `map_spin(steering_angle_deg, direction)`：4 个节臂恒为 0；4 个轮臂按对角线交替 ±幅度（|角|&lt;3°→0，最大 90°），幅度来自 `/my_robot/steering_wheel`。

测试方向盘输入：

```bash
ros2 topic pub /my_robot/motion_mode_switch std_msgs/msg/Int32 "{data: 0}" -1
ros2 topic pub /my_robot/track_width_switch std_msgs/msg/Int32 "{data: 0}" -1
ros2 topic pub /my_robot/steering_wheel std_msgs/msg/Float64 "{data: 30.0}" -r 20
ros2 topic echo /my_robot/steering_curvature
ros2 topic echo /my_robot/joint_commands
```

## 外界开关（需外部 pub 或后续 panel 节点）

| 话题 | 值 |
|------|-----|
| `/my_robot/motion_mode_switch` | 0=转向 1=顺时针自转 2=逆时针自转 |
| `/my_robot/track_width_switch` | 0=窄轮距 1=宽轮距 |
| `/my_robot/steering_wheel` | 有符号转向角（度），如 −30~+30 |

## FSM 状态（case 0~5）

| case | mode | 说明 |
|------|------|------|
| 0 | `narrow_track` | 窄轮距转向 |
| 1 | `switch_to_wide` | Action 切宽中 |
| 2 | `wide_track` | 宽轮距转向 |
| 3 | `switch_to_narrow` | Action 切窄中 |
| 4 | `spin_left` | 原地左转 |
| 5 | `spin_right` | 原地右转 |

## 转向关节标定

| 位置 | 关节名 | 类型 | 说明 |
|------|--------|------|------|
| 1 | `link_002_joint` | 节臂 | 右前节臂 |
| 2 | `link_003_joint` | 轮臂 | 右前轮臂 |
| 3 | `link_005_joint` | 节臂 | 左前节臂 |
| 4 | `link_006_joint` | 轮臂 | 左前轮臂 |
| 5 | `link_008_joint` | 节臂 | 左后节臂 |
| 6 | `link_009_joint` | 轮臂 | 右后轮臂 |
| 7 | `link_011_joint` | 节臂 | 右后节臂 |
| 8 | `link_012_joint` | 轮臂 | 左后轮臂 |

## 各 mode 关节角来源

| mode | 来源 |
|------|------|
| `narrow_track` / `wide_track` | `map(curvature, track_mode)` |
| `switch_to_*` | SetTrackWidth Action（固定窄/宽角） |
| `spin_left` / `spin_right` | `map_spin(steering_wheel°, direction)`，节臂 0、轮臂对角偏转 |

## 主要话题

| 话题 | 说明 |
|------|------|
| `/my_robot/steering_wheel` | 有符号转向角（度） |
| `/my_robot/steering_curvature` | 曲率 κ (1/m) |
| `/my_robot/mode` | FSM 当前 mode |
| `/my_robot/fsm_state` | case 0~5 |
| `/my_robot/joint_commands` | 8 腿目标角 |
| `/my_robot/joint_states` | 12 关节反馈 |
| `/my_robot/wheel_speed` | 四轮 rad/s |
| `/set_track_width` | 轮距切换 Action |

## 修改指南

| 目标 | 文件 |
|------|------|
| 转向角→曲率 | `ros2_ws/src/my_robot_maps/src/steering_angle_to_curvature.cpp` |
| 窄/宽映射链 | `ros2_ws/src/my_robot_maps/src/narrow_track_maps.cpp`, `wide_track_maps.cpp` |
| **map(κ→8腿)** | `ros2_ws/src/my_robot_maps/src/curvature_to_joints.cpp` |
| 腿控 / Action | `ros2_ws/src/leg_controller/src/leg_controller.cpp` |
| FSM | `ros2_ws/src/robot_fsm/src/robot_fsm.cpp` |
| 轮速 | `ros2_ws/src/mobility_bringup/src/mobility_node.cpp` |
| Webots IO | `ros2_ws/src/webots_ros2_bridge/src/ros2_bridge.cpp` |
