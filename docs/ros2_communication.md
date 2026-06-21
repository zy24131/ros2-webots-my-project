# ROS2 通信机制（窄轮距简化版）

## 使用的机制

- **Topic** — 全部控制数据流
- **Parameter** — `my_robot.yaml`（`wheel_speed`、`command_timeout_ms`）
- **Timer** — 各节点周期发布

未使用：**Action**、**Service**、**FSM 相关话题**。

## 主要话题

| 话题 | 发布 | 订阅 | 说明 |
|------|------|------|------|
| `/my_robot/steering_wheel` | 外部 | `steering_controller` | 转向角（度） |
| `/my_robot/steering_command` | `steering_controller` | `actuator_executor` | 8 关节目标 |
| `/my_robot/joint_commands` | `actuator_executor` | `hardware_bridge` | 最终关节命令 |
| `/my_robot/joint_states` | `hardware_bridge` | `actuator_executor` | 关节反馈 |
| `/my_robot/wheel_speeds` | `mobility_controller` | `hardware_bridge` | 四轮同速 |
| `/my_robot/wheel_speed` | `mobility_controller` | `hardware_bridge` | 兼容 |
| `/my_robot/system_ready` | `actuator_executor` | 可选 | 就绪标志 |

## 已移除（相对旧版）

| 话题 / 机制 | 说明 |
|-------------|------|
| `/my_robot/motion_mode_switch` | 无 FSM |
| `/my_robot/track_width_switch` | 无轮距切换 |
| `/my_robot/mode` | 无模式发布 |
| `/my_robot/fsm_state` | 无 FSM |
| `/my_robot/steering_curvature` | 不再单独发布 |
| `/set_track_width` Action | 已删除 |

## 数据流

```text
steering_wheel → steering_controller → steering_command
                                           ↓
                                    actuator_executor → joint_commands
mobility_controller → wheel_speeds → hardware_bridge
```

## 参数

`robot_fsm/config/my_robot.yaml`：

| 节点 | 参数 |
|------|------|
| `mobility_controller` | `wheel_speed`（默认 0.2 rad/s） |
| `actuator_executor` | `command_timeout_ms`（默认 450） |
