# my_project — ROS2 + Webots（窄轮距转向）

```text
my_project/
├── webots/
├── ros2_ws/src/
│   ├── my_robot_msgs/
│   ├── my_robot_maps/          # 窄轮距 al.cpp 算法
│   ├── hardware_bridge/
│   ├── steering_controller/    # 方向盘 → SteeringCommand
│   ├── actuator_executor/      # → joint_commands
│   ├── mobility_controller/    # 固定轮速
│   └── robot_fsm/              # launch + config（无 FSM 节点）
├── run_webots.sh
└── docs/
```

## 编译与运行

```bash
cd ~/ros2_webots/my_project/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select my_robot_msgs my_robot_maps hardware_bridge steering_controller actuator_executor mobility_controller robot_fsm
source install/setup.bash

# 终端1
cd ~/ros2_webots/my_project && bash run_webots.sh

# 终端2
ros2 launch robot_fsm my_robot.launch.xml
```

## 控制数据流（窄轮距 only）

```text
steering_wheel → steering_controller → steering_command → actuator_executor → joint_commands
mobility_controller → wheel_speeds（四轮同速）→ hardware_bridge
```

详见 [docs/steering_flow.md](docs/steering_flow.md)、[docs/ros2_communication.md](docs/ros2_communication.md)。

## 测试

```bash
ros2 topic pub /my_robot/steering_wheel std_msgs/msg/Float64 "{data: 30.0}" -r 20
ros2 topic echo /my_robot/joint_commands
ros2 topic echo /my_robot/wheel_speeds
```

## 修改指南

| 目标 | 文件 |
|------|------|
| 窄轮距算法 | `my_robot_maps/src/narrow_track_maps.cpp`, `curvature_to_joints.cpp` |
| 转向 | `steering_controller/` |
| 关节下发 | `actuator_executor/` |
| 轮速 | `mobility_controller/`（`wheel_speed` 参数） |
| Webots IO | `hardware_bridge/` |
