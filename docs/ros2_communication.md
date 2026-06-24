# ROS2 通信机制使用情况

本文说明 `my_project` 当前用了哪些 ROS2 通信方式、哪些没用，以及是否够用。

---

## 结论（先看这个）

**没有「全用上」ROS2 的所有通信机制，也不需要全用上。**

当前工程主要使用：

- **Topic（话题）** — 节点间连续数据流，主力
- **Service（服务）** — 模式/轮距意图切换（一问一答）
- **Action（动作）** — 轮距切换等「要等待完成」的长任务
- **Parameter（参数）** — launch + yaml 配置
- **Timer（定时器）** — 固定频率发布控制量

未使用：Lifecycle、Component、tf2、ros2_control 等（见下文）。

对**上位机 + Webots 仿真**栈来说，这样是合理且常见的。

---

## 已使用的 ROS2 机制

### 1. Topic（话题）— 主力

| 话题 | 类型 | 发布者 | 订阅者 | 说明 |
|------|------|--------|--------|------|
| `/my_robot/mode` | `String` | `robot_fsm` | `steering_controller`、`mobility_controller` | FSM 当前模式 |
| `/my_robot/fsm_state` | `Int32` | `robot_fsm` | 调试 | case 0~5 |
| `/my_robot/steering_wheel` | `Float64` | 外部 | `steering_controller` | 有符号转向角（度） |
| `/my_robot/steering_command` | `SteeringCommand` | `steering_controller` | `actuator_executor` | 8 关节目标 |
| `/my_robot/steering_curvature` | `Float64` | `steering_controller` | `mobility_controller` | 曲率 κ |
| `/my_robot/joint_commands` | `JointState` | `actuator_executor` | `hardware_bridge` | 8 腿最终命令（唯一发布者） |
| `/my_robot/joint_states` | `JointState` | `hardware_bridge` | `actuator_executor` | 12 关节反馈 |
| `/my_robot/wheel_speeds` | `WheelSpeeds` | `mobility_controller` | `hardware_bridge` | 四轮差速 rad/s |
| `/my_robot/wheel_speed` | `Float64` | `mobility_controller` | `hardware_bridge` | 兼容旧接口，四轮同速 |
| `/my_robot/system_ready` | `Bool` | `actuator_executor` | 可选监听 | joint_states 就绪后为 true |

### 2. Action（动作）— 1 个

| Action | 服务端 | 客户端 | 用途 |
|--------|--------|--------|------|
| `/set_track_width` | `actuator_executor` | `robot_fsm` | 窄/宽轮距切换，带 progress 反馈与 success 结果 |

定义文件：`my_robot_msgs/action/SetTrackWidth.action`

### 3. Service（服务）— 2 个

| Service | 服务端 | 用途 |
|---------|--------|------|
| `/my_robot/set_motion_mode` | `robot_fsm` | 0=转向 1=顺时针自转 2=逆时针自转 |
| `/my_robot/set_track_width_switch` | `robot_fsm` | 0=窄轮距 1=宽轮距（意图；实际伸缩走 Action） |

定义文件：`my_robot_msgs/srv/SetMotionMode.srv`、`SetTrackWidthSwitch.srv`

示例：

```bash
ros2 service call /my_robot/set_motion_mode my_robot_msgs/srv/SetMotionMode "{mode: 0}"
ros2 service call /my_robot/set_track_width_switch my_robot_msgs/srv/SetTrackWidthSwitch "{track_width: 1}"
```

### 4. 自定义消息

| 消息 | 文件 | 用途 |
|------|------|------|
| `SteeringCommand` | `msg/SteeringCommand.msg` | 8 关节角 + source + valid |
| `WheelSpeeds` | `msg/WheelSpeeds.msg` | 4 轮角速度 + valid |

### 5. Parameter（参数）

集中配置：`robot_fsm/config/my_robot.yaml`

| 节点 | 主要参数 |
|------|----------|
| `actuator_executor` | `position_tolerance`、`wide_steering_angle`、`command_timeout_ms` |
| `mobility_controller` | `wheel_speed`、`track_half_width_narrow`、`track_half_width_wide` |

Launch 通过 `<param from="..." path="节点名"/>` 加载。

### 6. Timer（定时器）

| 节点 | 周期 | 作用 |
|------|------|------|
| `steering_controller` | 20 ms | 发布 steering_command / curvature |
| `actuator_executor` | 20 ms | 合并命令、看门狗、Action 插值 |
| `mobility_controller` | 50 ms | 发布 wheel_speeds |
| `robot_fsm` | 50 ms | 状态机 Tick |

---

## 数据流简图

```text
外部输入 (Service / Float64)
        │
        ▼
   robot_fsm ────── Action ──────► actuator_executor ── Topic ──► hardware_bridge
        │ mode                           ▲ steering_command          (Webots / 未来 CAN)
        │                                │ joint_states
        ▼                                │
steering_controller ─────────────────────┘
        │ steering_curvature
        ▼
mobility_controller ── Topic (wheel_speeds) ──► hardware_bridge
```

---

## 未使用的 ROS2 机制

| 机制 | 当前状态 | 典型使用场景 |
|------|----------|--------------|
| **Service（服务）** | 已用 | 模式/轮距意图切换 |
| **Lifecycle（生命周期）** | 未用 | 严格启动顺序、休眠/唤醒 |
| **Component（组件化）** | 未用 | 多节点合并单进程，降低延迟 |
| **tf2** | 未用 | 坐标变换、SLAM、多传感器融合 |
| **ros2_control** | 未用 | 标准关节/控制器框架 |
| **QoS 精细配置** | 基本默认（队列深度 10） | 实时/可靠传输要求极高时 |
| **参数动态回调** | 未用 | 运行中改参并立即生效 |

这些不是「必须补齐」；按需求再加即可。

---

## Topic vs Action vs Service 怎么选（本项目实践）

| 场景 | 选用 | 本项目例子 |
|------|------|------------|
| 连续控制量、状态流 | **Topic** | 关节角、轮速、mode |
| 要等待完成、有进度 | **Action** | 轮距切换 SetTrackWidth |
| 偶发、一问一答 | **Service** | 模式切换 SetMotionMode / SetTrackWidthSwitch |

---

## 与 CAN / 下位机的关系

ROS2 通信**只在上位机进程之间**（或仿真桥接）使用。

真机 CAN **不是 ROS2 的一种通信**，需要 L1 驱动把 Topic 转成 CAN 帧：

```text
actuator_executor  --Topic-->  can_bridge（待开发）  --CAN-->  下位机 ECU
```

上层 ROS 节点接口可保持不变，不必为了 CAN 再换一种 ROS 通信类型。

---

## 真机阶段可选补充

| 需求 | 建议机制 |
|------|----------|
| 急停、复位 | Service |
| 启动顺序 / 休眠 | Lifecycle |
| 在线调参 | Parameter 回调 |
| 更低延迟 | Component 或单进程合并 |

---

## 相关文档

- [steering_flow.md](steering_flow.md) — 控制数据流
- [架构说明.md](架构说明.md) — 分层架构
- [README.md](../README.md) — 编译与运行
