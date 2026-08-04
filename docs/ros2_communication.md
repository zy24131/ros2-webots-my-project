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

### 1. 对外 Topic（HMI / 操作员）

| 话题 | 类型 | 发布者 | 订阅者 | 说明 |
|------|------|--------|--------|------|
| `/my_robot/mode` | `String` | `fsm` | `steering_control` | FSM 当前模式（case 0~5） |
| `/my_robot/steering_input` | `Float64` | 外部 | `steering_control` | 方向盘输入，有符号转向角（度） |
| `/my_robot/joint_states` | `JointState` | `hardware_bridge` | `actuator_control` | 12 关节反馈 |
| `/my_robot/imu` | `Imu` | `hardware_bridge` | 可选 | 模型中心 IMU（Webots） |

### 2. 内部 Topic（`/my_robot/internal/`，节点间专用）

| 话题 | 类型 | 发布者 | 订阅者 | 说明 |
|------|------|--------|--------|------|
| `/my_robot/internal/wheel_speeds` | `WheelSpeeds` | `wheel_speed_control` | `hardware_bridge` | 四轮轮速 rad/s（当前固定 0） |
| `/my_robot/steering_command` | `SteeringCommand` | `steering_control` | `actuator_control` | 8 关节目标 |
| `/my_robot/joint_commands` | `JointState` | `actuator_control` | `hardware_bridge` | 8 腿最终命令 |

### 2. Action（动作）— 1 个

| Action | 服务端 | 客户端 | 用途 |
|--------|--------|--------|------|
| `/set_track` | `actuator_control` | `fsm` | 窄/宽轮距切换，带 progress 反馈与 success 结果 |

定义文件：`my_robot_msgs/action/SetTrackAction.action`

### 3. Service（服务）— 2 个

| Service | 服务端 | 用途 |
|---------|--------|------|
| `/my_robot/set_motion` | `fsm` | 0=转向 1=顺时针自转 2=逆时针自转 |
| `/my_robot/set_track` | `fsm` | 0=窄轮距 1=宽轮距（Track 意图；伸缩走 SetTrackAction） |

定义文件：`my_robot_msgs/srv/SetMotion.srv`、`SetTrack.srv`

示例：

```bash
ros2 service call /my_robot/set_motion my_robot_msgs/srv/SetMotion "{mode: 0}"
ros2 service call /my_robot/set_track my_robot_msgs/srv/SetTrack "{track: 1}"
```

### 4. 自定义消息

| 消息 | 文件 | 用途 |
|------|------|------|
| `SteeringCommand` | `msg/SteeringCommand.msg` | 8 关节角 + source + valid |
| `WheelSpeeds` | `msg/WheelSpeeds.msg` | 4 轮角速度 + valid |

### 5. Parameter（参数）

唯一配置文件：`fsm/config/my_robot.yaml`  
Launch 与各节点通过 `<param from=".../my_robot.yaml"/>` 或 `--params-file` 加载；launch 中 `name=` 须与 yaml 顶层键一致（如 `steering_control`）。

共享参数放在 `/**:` 段（如 `wide_neutral_angle_deg`），各节点段只写本节点专有项。

| 节点 | 主要参数 |
|------|----------|
| `/**` | `wide_neutral_angle_deg`（steering_control、actuator_control 共用） |
| `actuator_control` | `position_tolerance`、`command_timeout_ms`、`track_step_ratio`、QoS |
| `steering_control` | `steering_deadband_deg`、QoS |
| 各节点 | `qos.<端点名>.profile` / `depth` 等 |

### 5.1 QoS（服务质量）

工具：`fsm::QoSFromParams()`（`fsm/include/fsm/qos.hpp`）

每个 pub/sub 在参数里用 `qos.<端点名>` 配置，例如：

```yaml
qos:
  steering_command_pub:
    profile: reliable
    depth: 10
```

| 字段 | 可选值 | 说明 |
|------|--------|------|
| `profile` | `default`、`sensor_data`、`reliable`、`best_effort`、`system_default` | 预设策略 |
| `depth` | 正整数 | 队列深度 |
| `reliability` | `best_effort`、`reliable`、空 | 覆盖 profile 的可靠性 |
| `durability` | `volatile`、`transient_local`、空 | 持久性 |
| `history` | `keep_last`、`keep_all`、空 | 历史策略 |

默认策略（发布者与订阅者需匹配）：

| 话题类型 | 默认 profile | 原因 |
|----------|--------------|------|
| `mode`、`steering_command`、`joint_commands` | `reliable` | 状态/控制，不能丢 |
| `steering_input`、`joint_states`、`wheel_speeds` | `sensor_data` | 高频流，取最新即可 |

运行时覆盖示例：

```bash
ros2 run steering_control steering_control_node --ros-args \
  -p qos.steering_input_sub.profile:=reliable \
  -p qos.steering_input_sub.depth:=5
```

查看实际 QoS：`ros2 topic info /my_robot/joint_commands -v`

### 6. Timer（定时器）

| 节点 | 周期 | 作用 |
|------|------|------|
| `steering_control` | 20 ms | 发布 steering_command |
| `actuator_control` | 20 ms | 合并命令、看门狗、Action 插值 |
| `wheel_speed_control` | 50 ms | 发布 wheel_speeds |
| `fsm` | 50 ms | 状态机 Tick |

---

## 数据流简图

```text
外部输入 (Service / Float64)
        │
        ▼
   fsm ────── Action ──────► actuator_control ── Topic ──► hardware_bridge
        │ mode                           ▲ steering_command          (Webots / 未来 CAN)
        │                                │ joint_states
        ▼                                │
steering_control ─────────────────────┘
        │
        ▼
wheel_speed_control ── Topic (wheel_speeds) ──► hardware_bridge
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
| **QoS 精细配置** | 已用（参数可配） | 控制/状态可靠，传感器流 best effort |
| **参数动态回调** | 未用 | 运行中改参并立即生效 |

这些不是「必须补齐」；按需求再加即可。

---

## Topic vs Action vs Service 怎么选（本项目实践）

| 场景 | 选用 | 本项目例子 |
|------|------|------------|
| 连续控制量、状态流 | **Topic** | 关节角、轮速、mode |
| 要等待完成、有进度 | **Action** | 轮距切换 SetTrackAction |
| 偶发、一问一答 | **Service** | Motion：SetMotion / Track：SetTrack |

---

## 与 CAN / 下位机的关系

ROS2 通信**只在上位机进程之间**（或仿真桥接）使用。

真机 CAN **不是 ROS2 的一种通信**，需要 L1 驱动把 Topic 转成 CAN 帧：

```text
actuator_control  --Topic-->  can_bridge（待开发）  --CAN-->  下位机 ECU
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
