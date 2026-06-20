# 转向控制数据流

```text
/my_robot/steering_wheel (°)
        │
        ▼
steering_controller ──► /my_robot/steering_command (SteeringCommand)
        │                 /my_robot/steering_curvature
        ▼
actuator_executor ──► /my_robot/joint_commands  (唯一发布者，450ms 看门狗)
        ▲
        │ SetTrackWidth Action（轮距切换）
robot_fsm ──► /my_robot/mode

/my_robot/steering_curvature
        ▼
mobility_controller ──► /my_robot/wheel_speeds（曲率差速）
        │                 /my_robot/wheel_speed（兼容）
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
