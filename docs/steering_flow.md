# 转向控制数据流（窄轮距 only）

```text
/my_robot/steering_wheel (°)
        │
        ▼
steering_controller ──► /my_robot/steering_command
        ▼
actuator_executor ──► /my_robot/joint_commands  (450ms 看门狗)
        ▲
        │ joint_states
        │
mobility_controller ──► /my_robot/wheel_speeds（四轮同速）
        ▼
hardware_bridge (Webots)
```

## 节点

| 节点 | 职责 |
|------|------|
| `steering_controller` | 窄轮距 map() → SteeringCommand |
| `actuator_executor` | → joint_commands |
| `mobility_controller` | 固定 wheel_speed |
| `hardware_bridge` | Webots IO |

已移除：FSM、宽轮距、自转、SetTrackWidth Action。

## 就绪

`actuator_executor` 收到 8 路 `joint_states` 后发布 `/my_robot/system_ready=true`。
