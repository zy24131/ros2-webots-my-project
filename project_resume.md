# mower_fsm 项目简历（关键模块）

> Raptor / Cooper 割草机业务层（`fsm_node`）。按「谁跟谁说话、状态怎么转」归纳，对应本仓库核心文件。

---

## 和外部模块怎么通信

1. **运动规划（Planner）和感知定位（Perception）分开调，不混在一个接口里：**
   - **Planner**：`PlannerManager` 通过 `PlannerAction` 连 `planner_action_server`，负责出桩、校准、割草、回桩等**怎么动**
   - **Perception**：`PerceptionManager` 通过 `MsfAction` / `VmapAction` / `RtkAction` 等，负责设桩位、定位、Vmap 等**位姿从哪来**
   - 举例：方向校准时先调 Perception 设桩位、再调 Planner 跑 CALIBRATION mission——两步分开，各找各的 Manager

2. **三种 ROS/蓝牙用法：**
   - **Topic**：持续收状态（mission 进度、planner 状态、MSF 状态、边界是否有效等）
   - **Action**：发命令等结果（Planner / 感知的长任务，走 `runActionTask(function_type, …)`）
   - **蓝牙**：APP 发意图（如 0x12 方向校准、退桩、BSR），`BluetoothManager` 转成 `GlobalFsm::fire(MowerTrigger::…)`

3. **名字和命令字集中写在 `config.h`**：Topic 名、Planner 模式（`PLANNER_MODE_*`）、Planner 命令号（`PLANNER_*`）、蓝牙 CMD，避免各文件自己写字符串。

**关键文件**

| 文件 | 干什么 |
|------|--------|
| `include/mower_fsm/config.h` | Topic、模式、命令字、蓝牙 CMD、枚举 |
| `action/Planner.action` | Planner Action 消息格式 |
| `planner_manager.h` / `.cpp` | 调 Planner：`runMission`、`runActionTask`、取失败原因 |
| `perception_manager.h` / `.cpp` | 调感知：MSF / Vmap / RTK 等 |
| `bluetooth_manager.h` / `.cpp` | 和 APP 蓝牙通信、上报错误/进度 |
| `include/serial_comm/aprGxHost/ctrlMapApp.h` | 蓝牙 CMD 定义（和 APP 对齐） |

---

## 业务代码怎么组织

1. **一个大业务一块 Manager**（`MapManager`、`MowManager`、`RobotStatusManager`、`TaskManager` 等），各自管一块事。`MapManager` 只管建图/校准/BSR 的具体步骤，**不持有状态机**，方便被多个 FSM 状态复用。

2. **状态机管「在第几步」，MapManager 管「这一步做什么」：**
   - 外层：`mower_state.cpp` 里 `DirectionInit`、`ExitChargePile` 等，用定时器每隔几百 ms 推进一步
   - 内层：例如 `direction_init_state_` 从 START → MOVING → DONE → TURN_AROUND，每步调 `buildMapDirectionInitStart/Moving/Done/TurnAround`

3. **开机 Init**：`CheckModuleStatusChain` 等 planner、perception 都 ready 了，才让 FSM 往下走，避免算法还没起来就发 mission。

**关键文件**

| 文件 | 干什么 |
|------|--------|
| `map_manager.h` / `.cpp` | 校准四步、BSR 退桩五步、报错、地图读写 |
| `mower_state.h` / `.cpp` | 各 FSM 状态的 onEnter、定时 loop、onExit |
| `chain_functions.h` / `.cpp` | 开机模块就绪检查链 |
| `module_callback.h` | 模块名、就绪回调 |
| `main_node.cpp` | `fsm_node` 入口，初始化各 Manager |

---

## 状态机与 Trigger

1. **总状态机 `GlobalFsm`**（`stateless4cpp`）：状态是 `MowerState`，切换靠 `MowerTrigger`，配置全在 `global_state_machine.cpp::configStates()`。蓝牙、定时器等异步事件进队列，串行 `fire()`，避免多线程同时改状态。

2. **建图里和「出桩/校准」相关的三个状态，同一时刻只会进一个。** 先分「进哪条业务线」，再在业务线里选具体状态：

**第一步：进建图还是进 BSR？（两条线互斥）**

| 目标状态 | Trigger | 谁发 | 前置条件（当前 GlobalFsm 状态） |
|----------|---------|------|--------------------------------|
| 进入建图子树 | `START_BUILD_RAPTOR_MAP` | APP / IoV 进地图编辑 | `IDLE` / `CHARGING` / `EXIT_CHARGE_PILE` |
| 进入 BSR 子树 | `ENTER_BASE_STATION_RECOMMEND` | APP BSR 命令 `ENTER_BSR_MODE` | `IDLE` / `CHARGING` |

在 BSR 线里只会出现 **`BASE_STATION_RECOMMEND_EXIT_PILE`**；在建图线里才会出现 **`DIRECTION_INIT`** 或 **`EXIT_CHARGE_PILE`**。

---

**第二步 A：建图线 —— 进校准还是退桩？**

| 进哪个 | Trigger | 谁发 | 判断条件 |
|--------|---------|------|----------|
| **`DIRECTION_INIT`** | `START_DIRECTION_INIT` | APP 蓝牙 **0x12** | 当前状态必须是 **`BUILD_RAPTOR_MAP`** 或已在 **`DIRECTION_INIT`**；`notifyExitPileToEcu` 后 fire |
| **`DIRECTION_INIT`** | `START_AUTO_MAP_DIRECTION_INIT` | 自动建图 `DropAndMowManager::recvStartAutoMapping` | ① `checkAutoMapCondOnReceivingCommand` 通过（非夜间、电量/网络 OK 等）② **`getOnChargePile()`** ③ **`checkMapExists()` 为 false**（本地还没有地图） |
| **`EXIT_CHARGE_PILE`** | `START_EXIT_PILE` | APP 蓝牙 **退桩**（`COMM_CONTENT_APP_OUT_CHARGE_PILE`） | 代码里**不查有没有地图**，由 APP 决定点退桩；GlobalFsm 仅允许从 **`BUILD_RAPTOR_MAP` / `WAIT_FOR_BUILD_MAP`** 转入（在其他状态 fire 无效） |
| **`EXIT_CHARGE_PILE`** | `START_AUTO_MAP_EXIT_PILE` | 自动建图同上 | ① 自动建图条件通过 ② **在桩** ③ **`checkMapExists()` 为 true**（已有地图） |
| 都不进，直接探索 | `RELOCATE_AUTO_MAP` | 自动建图 | 条件通过但 **不在桩** → 跳过出桩/校准 |

手动建图时 **APP 用 0x12 还是退桩命令** 区分校准/退桩；自动建图时 **代码用 `checkMapExists()`** 区分。

`CHARGING` 状态下可直接 fire 自动建图的两个 Trigger（`START_AUTO_MAP_*`），不必先进 `BUILD_RAPTOR_MAP`。

---

**第二步 B：BSR 线 —— 退桩**

| 进哪个 | Trigger | 谁发 | 判断条件 |
|--------|---------|------|----------|
| **`BASE_STATION_RECOMMEND_EXIT_PILE`** | `START_BASE_STATION_RECOMMEND_EXIT_PILE` | APP BSR 命令 **`START_EXIT_PILE`（cmd 5）** | 当前状态为 **`BASE_STATION_RECOMMEND`** 或已在 **`BASE_STATION_RECOMMEND_EXIT_PILE`** |

---

**决策树（简版）**

```
用户在 APP 的操作
│
├─ 进「基站推荐」且 IDLE/CHARGING ──► BASE_STATION_RECOMMEND
│       └─ BSR 点退桩 ──► BASE_STATION_RECOMMEND_EXIT_PILE
│
└─ 进「建图/地图编辑」且 IDLE/CHARGING ──► BUILD_RAPTOR_MAP
        │
        ├─ APP 0x12 方向校准 ──► DIRECTION_INIT
        ├─ APP 退桩命令 ──► EXIT_CHARGE_PILE
        │
        └─ 自动建图（DropAndMow，在桩且条件 OK）
                ├─ 无地图 ──► DIRECTION_INIT
                ├─ 有地图 ──► EXIT_CHARGE_PILE
                └─ 不在桩 ──► RELOCATE_AUTO_MAP（不经过上述两态）
```

3. **Planner 失败原因**：统一用 `getPlannerMissionFailedReason(mode)` 向 planner 要问失败码，再通过蓝牙 `sendDirectionErrorStateToApp`（0x001A 原因）和 `sendDirectionErrorToApp`（通知 APP 流程结束）发给手机。

**关键文件**

| 文件 | 干什么 |
|------|--------|
| `global_state_machine.h` / `.cpp` | 状态图、`GlobalFsm::fire` |
| `fire_event.h` | 异步 Trigger 事件 |
| `include/stateless4cpp/` | 状态机库 |
| `mower_state.h` / `.cpp` | `DirectionInit::directionloop` 等 |
| `drop_and_mow_manager.cpp` | 自动建图：有地图退桩，没地图走校准 |

---

## 方向校准失败怎么报到 APP（代表流程）

1. `DirectionInit` 里子状态任一步失败 → `buildMapDirectionError()`。

2. 向 Planner 要失败码：`getPlannerMissionFailedReason(..., CALIBRATION)`，读 `result->data[2]`。若是 GPS 弱、没充电、按 STOP 等本地错误，可能还没 planner 失败码，此时 reason 可能是 0。

3. 发给 APP：
   - 失败原因：`sendDirectionErrorStateToApp` → 蓝牙动态数据 **0x001A**
   - 流程结束：`sendDirectionErrorToApp`
   - 成功时的进度：方向校准用 **0xCB**；建图退桩/BSR 用出桩进度那条

**关键文件**：`map_manager.cpp`、`mower_state.cpp`、`planner_manager.cpp`、`bluetooth_manager.cpp`

---

## 编译与车型

- `CMakeLists.txt` 里 `COOPER` / `MR813` / `C4210` 区分 Cooper 和不同芯片平台。
- Cooper 差异多在参数、工厂脚本、报警 bit、IoT 能力；FSM 主流程共用。

---

## 更多细节

| 文档 | 内容 |
|------|------|
| [map_manager_state_machine_and_exit_pile_error.md](./map_manager_state_machine_and_exit_pile_error.md) | 三个出桩状态何时进、DirectionInit 逐步逻辑、错误码 FAQ |
| [README.md](./README.md) | 本目录说明 |
