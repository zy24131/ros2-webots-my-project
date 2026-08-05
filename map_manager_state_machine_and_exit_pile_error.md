# MapManager 状态机 × 退桩/校准错误上报 — 项目问答

> 围绕 `src/mower_fsm/map_manager.cpp` 中的子状态机与 `sendDirectionErrorStateToApp` 整理。  
> 关联文件：`mower_state.cpp`、`planner_manager.cpp`、`bluetooth_manager.cpp`、`include/mower_fsm/config.h`

---

## 一、架构总览

### Q1：到底有几个状态机？分别啥时候执行？为啥有三个？

**不是同时跑三个**，而是 **3 个 GlobalFsm 子状态** 对应 **3 种业务场景**（出桩/校准动作相似，但触发条件、Planner 模式、成功后去向不同）。另加 **1 套 MapManager 遗留代码**，当前无人调用。

#### 三个「在跑」的子状态（互斥，同一时刻只进一个）

| GlobalFsm 状态 | 驱动位置 | 何时进入 | MapManager 参与方式 | Planner 模式 |
|----------------|----------|----------|---------------------|--------------|
| **`DIRECTION_INIT`** | `DirectionInit::directionloop()`（160ms） | 建图模式下 APP 发 **0x12 方向校准**；或自动建图且无地图 → `START_AUTO_MAP_DIRECTION_INIT` | ✅ 逐步调 `buildMapDirectionInitStart/Moving/Done/TurnAround` | `PLANNER_MODE_CALIBRATION`（101） |
| **`EXIT_CHARGE_PILE`** | `ExitChargePile::loopCheckFinish()`（200ms） | 建图模式下 APP 发 **退桩**（`COMM_CONTENT_APP_OUT_CHARGE_PILE`）→ `START_EXIT_PILE`；或自动建图且**已有地图** → `START_AUTO_MAP_EXIT_PILE` | ⚠️ 几乎不在 map_manager 逐步驱动；`onEnter` 里 `runMission(DOWN_PILE)`，pause/resume 用 `buildMapExitPile*` | `PLANNER_MODE_DOWN_PILE`（7） |
| **`BASE_STATION_RECOMMEND_EXIT_PILE`** | `BaseStationRecommendExitPile::exitPileCallback()`（500ms） | 基站推荐（BSR）流程里 APP 发 BSR 退桩 → `START_BASE_STATION_RECOMMEND_EXIT_PILE` | ✅ 逐步调 `baseStationRecommendExitPile*Proc()` | `PLANNER_MODE_CALIBRATION`（101） |

#### 为啥要有三个？

物理动作都像「出桩 → 直线 → 调头」，但业务上下文不同：

| | ① 方向校准 | ② 建图退桩 `EXIT_CHARGE_PILE` | ③ BSR 退桩 |
|--|-----------|------------------------------|-----------|
| 父状态 | `BUILD_RAPTOR_MAP` | `BUILD_RAPTOR_MAP` | `BASE_STATION_RECOMMEND` |
| 典型前提 | 首次建图、需写桩位 | **已有地图**，只需从桩上下来 | 基站推荐录桩 |
| 成功后 | 录边界 / 自动建图_relocate | `GO_WAIT_FOR_BUILD_MAP` | `BASE_STATION_RECOMMEND_RECORD` |
| 进度蓝牙 | `sendDirectionInitProgressToApp` | `sendEditDirectionInitProgressToApp` | `sendEditDirectionInitProgressToApp` |
| 失败上报 | `MapManager::buildMapDirectionError()` | `mower_state.cpp` 里直接调 `sendDirectionErrorStateToApp` | `baseStationRecommendExitPileErrorProc()` |

**② 不能合并进 ①**：已有地图时不需要再做 PileConfig/写桩那一套，Planner 用的是 **DOWN_PILE(7)** 而不是 CALIBRATION。

最开始手动建图

```cpp
    void recv_enter_map_record(StampedBasicFrame_ *frame) {
        LOG_DEBUG<< "COMM_CONTENT_APP_ENTER_MAP_RECORD, " << intToHex(COMM_CONTENT_APP_ENTER_MAP_RECORD);

        char val[frame->data_len] = {0};
        memcpy(val, frame->data, frame->data_len);
        LOG_DEBUG<< "recv data from app: " << intToHex((uint8_t*)val, frame->data_len);

        MowerState state = GlobalFsm::instance().getCurrentState();
        LOG_DEBUG<<"current state: "<< State2Str(state);
        GlobalFsm::instance().fire(MowerTrigger::START_BUILD_RAPTOR_MAP);
    }
```



#### 建图场景时间轴（在桩上）

```
IDLE / CHARGING
      │
      ▼ START_BUILD_RAPTOR_MAP //手动建图
 BUILD_RAPTOR_MAP          ← 手动建图入口，不会自动进 WAIT
      │
      ├─ APP 0x12 ──────────────────► DIRECTION_INIT
      │                                      │
      │                                 校准成功（还在 DIRECTION_INIT）
      │                                      │
      │                                 APP 开始录元素
      │                                      ▼
      │                              FINISH_DIRECTION_INIT
      │                                      │
      │                                      ▼
      │                               RECORD_ELEMENT
      │
      ├─ APP 退桩 ───────────────────► EXIT_CHARGE_PILE
      │                                      │
      │                                 退桩成功
      │                                      ▼
      │                              GO_WAIT_FOR_BUILD_MAP
      │                                      │
      │                                 WAIT_FOR_BUILD_MAP
      │                                      │
      │                                 APP 开始录元素
      │                                      ▼
      │                              START_RECORD_ELEMENT
      │                                      │
      │                                      ▼
      │                               RECORD_ELEMENT
      │
      └─ 已在桩外 / APP 直接录 ───────► START_RECORD_ELEMENT
                                            （从 BUILD_RAPTOR_MAP 或 WAIT_FOR_BUILD_MAP）
                                             ▼
                                      RECORD_ELEMENT
                              
                              
（BSR 是独立入口，不与上面同时进）
  BASE_STATION_RECOMMEND ──► ③ BASE_STATION_RECOMMEND_EXIT_PILE ──► BSR 录基站
```

#### 第四个：`buildMapDirectionInitProc()`（遗留，❌ 未使用）

`map_manager.cpp` 里还有一套与 ① 相同的 `directionInitTypeDef` 状态机，写在 **`while(true)` 阻塞循环**里。早期实现；后来改为 `DirectionInit::directionloop()` 定时器逐步调 MapManager 函数。**全仓库无调用方**，可忽略。

#### ① 方向校准流程（补充）

```
APP 0x12 → fire(START_DIRECTION_INIT) → DIRECTION_INIT
         → directionloop() 每 160ms
         → buildMapDirectionInitStart / Moving / Done / TurnAround
         → 失败：buildMapDirectionError() → sendDirectionErrorStateToApp()
```

---

### Q2：`DirectionInit` 状态机执行逻辑

`BaseState` 

```cpp
namespace mower_fsm{

    enum class MowerState;
    enum class MowerTrigger;

    using namespace stateless4cpp;
    class BaseState{
    public:
        BaseState() = default;   // 默认构造函数
        virtual ~BaseState() = default; // 虚析构函数（允许子类析构）
        virtual void onEnter(stateless4cpp::SPtrTransition<MowerState,MowerTrigger> transition,stateless4cpp::SPtrParam param) = 0;// 纯虚函数：进入状态时调用
        virtual void onExit(stateless4cpp::SPtrTransition<MowerState,MowerTrigger> transition) = 0;// 纯虚函数：退出状态时调用
    };
    
}
```

`DirectionInit` 

```cpp
    class DirectionInit : public BaseState{
    public:
        void onEnter(SPtrTransition<MowerState,MowerTrigger> transition,SPtrParam param);
        void onExit(SPtrTransition<MowerState,MowerTrigger> transition);
    private:
        void directionloop();        // 方向初始化循环逻辑
        bool checkGPS();             // 检查GPS信号
    private:
        MowerTrigger trigger_;       // 触发的下一个状态转换
        ros::Timer direction_timer_; // ROS定时器，用于周期性执行初始化
        directionInitTypeDef direction_init_state_;  // 初始化子状态机
        std::atomic_bool loopquitflag_;  // 原子布尔，退出循环标志
        int loopcnt_;                 // 循环计数
    };
```

`DirectionInit` 是 **GlobalFsm 子状态**（`MowerState::DIRECTION_INIT`）

```cpp
// 构造函数中创建
SharedConfig _shared_config = std::make_shared<StateMachineConfig<MowerState, MowerTrigger>>();
```

```cpp
// global_state_machine.cpp 中配置    链式调用（Fluent API / Method Chaining）
SPtrBaseState directionInitState = std::make_shared<DirectionInit>();
_shared_config->config(MowerState::DIRECTION_INIT)      // 注册一个状态
    ->onEntry(makeEnterAction(directionInitState))     // 进入时执行 DirectionInit::onEnter()
    ->onExit(makeExitAction(directionInitState))       // 退出时执行 DirectionInit::onExit()
    ->subStateOf(MowerState::BUILD_RAPTOR_MAP)         // 是 BUILD_RAPTOR_MAP 的子状态
    ->permit(MowerTrigger::FINISH_DIRECTION_INIT, MowerState::RECORD_ELEMENT);  // 完成后转换到 RECORD_ELEMENT
```

```tex
GlobalFsm (总状态机)
└── MowerState::BUILD_RAPTOR_MAP (建图状态)
    └── MowerState::DIRECTION_INIT (子状态) ← DirectionInit 类实现
        ├── onEnter() → 执行方向初始化逻辑（checkGPS、directionloop）
        └── onExit()  → 退出时清理
```

```cpp
// 在 BUILD_RAPTOR_MAP 状态下
_shared_config->config(MowerState::BUILD_RAPTOR_MAP)
    ->permit(MowerTrigger::START_DIRECTION_INIT, MowerState::DIRECTION_INIT)  // ← 这里！

// 在 WAIT_FOR_BUILD_MAP 状态下
_shared_config->config(MowerState::WAIT_FOR_BUILD_MAP)
    ->permit(MowerTrigger::START_DIRECTION_INIT, MowerState::DIRECTION_INIT)  // ← 这里！
```

`BUILD_RAPTOR_MAP` 是啥角色？

就是进建图后的第一站：模式开好，APP 让你选：

- 没地图 → 去 DIRECTION_INIT
- 有地图 → 去 EXIT_CHARGE_PILE

                        ┌─ DIRECTION_INIT ──→ 录边
                        │   （首次/校准）
      进建图 ──→ BUILD_RAPTOR_MAP ─┤
                        │
                        └─ EXIT_CHARGE_PILE ──→ WAIT_FOR_BUILD_MAP ──→ 录边
                            （退桩）              （等着）


```
进建图 → BUILD_RAPTOR_MAP
    ↓
APP 发 0x12 → recv_direction_init → fire(START_DIRECTION_INIT)
    ↓
DIRECTION_INIT → onEnter 开始校准
```



```
用户发起自动建图 → DropAndMowManager.recvStartAutoMapping()
    ├── 在桩上 + 没地图 → 方向初始化 ← START_AUTO_MAP_DIRECTION_INIT
    ├── 在桩上 + 有地图 → 直接出桩
    └── 不在桩上 → 直接重定位
```

```
┌─────────────────────────────────────────────────────────┐
│  自动建图触发                                           │
│  DropAndMowManager.recvStartAutoMapping()               │
│      │                                                  │
│      ▼ fire(START_AUTO_MAP_DIRECTION_INIT)              │
│  DirectionInit ──完成──→ fire(RELOCATE_AUTO_MAP) ──→  │
│                                              AutoMapRelocate
└─────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────┐
│  普通模式触发                                           │
│  BluetoothManager（App 指令）                           │
│      │                                                  │
│      ▼ fire(START_DIRECTION_INIT)                       │
│  DirectionInit ──完成──→ 切遥控模式                     │
└─────────────────────────────────────────────────────────┘
```



| 自动建图 | `DropAndMowManager`（放草割草管理器） |
| -------- | ------------------------------------- |
| 普通模式 | `BluetoothManager`（蓝牙/App 指令）   |

```
【1】mower_state.cpp:540
    GlobalFsm::fire(RELOCATE_AUTO_MAP)

【2】global_state_machine.cpp:937-939
    mHandler->post(GlobalFsmTriggerRunnable)

【3】global_state_machine.cpp:969-971
    _sptr_state_machine->fire(RELOCATE_AUTO_MAP, param)

【4】state_machine.h:88-112  publicFire
    │
    ├─ getState() == DIRECTION_INIT
    ├─ getCurrentRepresentation()  → 拿到 DIRECTION_INIT 那张表
    ├─ tryFindHandler(RELOCATE_AUTO_MAP)  → ③ 里登记的 permit，找到
    ├─ destination = AUTO_MAP_RELOCATE
    │
    ├─ exit(transition)  ──────────────── 下面【5】
    ├─ setState(AUTO_MAP_RELOCATE)
    └─ enter(transition)  → AutoMapRelocate::onEnter

【5】state_representation.h:105-108, 131-134
    exit(transition)
      → executeExitActions(transition)
           → exit_actions_[0]->doIt(transition)   // 就是 ② 挂的 MowerExitAction1

【6】mower_state.h:738-739
    MowerExitAction1::doIt
      → directionInitState->onExit(transition)   // DirectionInit::onExit()
```

```
// 1. 你写的配置
onExit(makeExitAction(directionInitState))

// 2. makeExitAction 返回
MowerExitAction1(directionInitState)  // 包装成 Action 对象

// 3. 状态切换时，框架调用
action->doIt(transition);

// 4. MowerExitAction1::doIt 内部
_state->onExit(transition);  // ← 调用真正的 onExit
```







#### 两层结构

| 层级 | 类型 | 变量/类 | 周期 |
|------|------|---------|------|
| 外层 | GlobalFsm 状态 | `DirectionInit`（`BUILD_RAPTOR_MAP` 的子状态） | 进入/退出各一次 |
| 内层 | 子状态机 | `direction_init_state_`（`directionInitTypeDef`） | `directionloop()` 每 **160ms**（调头阶段改为 **50ms**） |

子状态枚举（`config.h`）：

```c
DIRECTION_INIT_START → DIRECTION_INIT_MOVING → DIRECTION_INIT_DONE
    → DIRECTION_TURN_AROUND → DIRECTION_INIT_FINISHED
任意步骤失败 → DIRECTION_INIT_ERROR
```

#### 进入条件（`onEnter`）

| Trigger | 来源 | 机器状态 |
|---------|------|----------|
| `START_DIRECTION_INIT` | APP 蓝牙 **0x12**（`COMM_CONTENT_APP_DIRECTION_INIT`） | `AM_STATUS_SUB_MODE_DIRECT_INIT` |
| `START_AUTO_MAP_DIRECTION_INIT` | 自动建图且无地图（`DropAndMowManager`） | `AM_STATUS_SUB_MODE_AUTO_MAP_RELOCATE_FOR_EXPLORE` |

进入时还会：`notifyExitPileToEcu`（蓝牙路径）、清 STOP 标志、关 RTK 原点 GPS 标志、播开始语音、启动 **160ms** 定时器，`direction_init_state_ = DIRECTION_INIT_START`。

#### `directionloop()` 每轮前置检查

在 `switch (direction_init_state_)` 之前：

1. **蓝牙断开**（且非自动建图）→ 调 `buildMapDirectionInitPauseProc()`，定时器改 **1s**，本轮回合直接 return（不转 ERROR）
2. **`loopquitflag_`** 为 true → 停定时器
3. **用户按 STOP**（`getPressStopDuringExitChargePile()`）→ 强制 `DIRECTION_INIT_ERROR`

#### 内层状态流转与 MapManager 调用

```
                    ┌─────────────────────────────────────┐
                    │  DIRECTION_INIT_START               │
                    │  checkGPS()                         │
                    │  buildMapDirectionInitStart()       │
                    └──────────────┬──────────────────────┘
                                   │ 0
                                   ▼
                    ┌─────────────────────────────────────┐
                    │  DIRECTION_INIT_MOVING              │
                    │  buildMapDirectionInitMoving()      │
                    │  loopcnt_>100 → ERROR              │
                    └──────────────┬──────────────────────┘
                          0        │        <0
                                   ▼
                    ┌─────────────────────────────────────┐
                    │  DIRECTION_INIT_DONE                │
                    │  buildMapDirectionInitDone()        │
                    │  成功：上报桩位、timer→50ms         │
                    │  loopcnt_>100 → ERROR              │
                    └──────────────┬──────────────────────┘
                                   │ 0
                                   ▼
                    ┌─────────────────────────────────────┐
                    │  DIRECTION_TURN_AROUND              │
                    │  buildMapDirectionTurnAround()      │
                    │  返回 1：保持本态；0：→ FINISHED     │
                    └──────────────┬──────────────────────┘
                                   │ 0
                                   ▼
                    ┌─────────────────────────────────────┐
                    │  DIRECTION_INIT_FINISHED            │
                    └─────────────────────────────────────┘

  任意失败 ─────────────────────────► DIRECTION_INIT_ERROR
                                      buildMapDirectionError()
                                      sendDirectionErrorStateToApp()
```

| 子状态 | MapManager 函数 | 返回值 → 下一态 |
|--------|-----------------|-----------------|
| `START` | `buildMapDirectionInitStart()` | `0`→MOVING；`-1` 或 GPS 弱→ERROR |
| `MOVING` | `buildMapDirectionInitMoving()` | `0`→DONE；`1` 保持；`<0` 或超时→ERROR |
| `DONE` | `buildMapDirectionInitDone()` | `0`→TURN_AROUND；`-1` 或超时→ERROR |
| `TURN_AROUND` | `buildMapDirectionTurnAround()` | `0`→FINISHED；`1` 保持；`<0`→ERROR |

**Planner 模式**：全程 `PLANNER_MODE_CALIBRATION`（101）。  
**进度蓝牙**：`sendDirectionInitProgressToApp`（0xCB，仅方向校准用）。

#### 各 MapManager 步骤在做什么（简要）

| 步骤 | 主要动作 |
|------|----------|
| `Start` | 校验在桩充电 → 感知 `setChargePilePosition` + `initializeLocalization` → `setModeSyncByAction(CALIBRATION)` |
| `Moving` | 轮询感知定位 + planner mission 进度 → 上报进度；感知/planner 失败返回 `-1` |
| `Done` | 取桩定位 → `PLANNER_PILE_CONFIG` / 建桩子图 / 存图 → `setCalParam(1)` 允许调头 |
| `TurnAround` | 等调头 mission 成功 → `setModeSync(IDLE)` → `setDirectionInitSucc(true)` → 进度 100% |

#### 成功收尾（`DIRECTION_INIT_FINISHED`）

| 场景 | 动作 |
|------|------|
| 手动建图（`START_DIRECTION_INIT`） | `buildMapNoneProc()` → planner 遥控模式 → 成功语音；**不自动 fire FSM**（等 APP 录边界） |
| 自动建图（`START_AUTO_MAP_DIRECTION_INIT`） | `fire(RELOCATE_AUTO_MAP)` → 进入自动建图_relocate |

校准成功标记：`setDirectionInitSucc(true)`，后续蓝牙 ACK 时 `sendDirectionInitSuccToApp()`。

#### 失败收尾（`DIRECTION_INIT_ERROR`）

1. `MapManager::buildMapDirectionError()` → `getPlannerMissionFailedReason(CALIBRATION)` → **`sendDirectionErrorStateToApp`** + `sendDirectionErrorToApp`
2. planner 切遥控、停车
3. 自动建图额外：`fire(ANY_TO_IDLE)` + 云端 `AM_CLOUD_DATA_EVENT_TYPE_DM_EXIT_DIRECINIT_FAIL`

本地失败（未充电、GPS 弱、按 STOP、循环超时）也可能进 ERROR，此时 planner 可能尚未 failed，`fail_reason` 可能为 **0**。

#### 暂停 / 恢复

| APP 命令 | 处理 |
|----------|------|
| 0x20 暂停 | `buildMapDirectionInitPauseProc()` → `pauseMissionByAction()`，状态 `DIRECT_PAUSE` |
| 0x21 恢复 | 仅在 `DIRECTION_INIT` 下 `buildMapDirectionInitResumeProc()` → `resumeMissionByAction()` |
| 蓝牙断开 | `directionloop` 内自动 pause（非自动建图） |

#### 退出（`onExit`）

停定时器、`setModeSyncByAction(REMOTE_CONTROL)`；若非 `RELOCATE_AUTO_MAP` 退出则清自动建图标志。

#### 与 GlobalFsm 的衔接

- 父状态：`BUILD_RAPTOR_MAP`（`subStateOf`）
- 可转出：`FINISH_DIRECTION_INIT`→录边界、`RELOCATE_AUTO_MAP`、`ANY_TO_IDLE`、`CHARGE_ON` 等（见 `global_state_machine.cpp`）

> 内层枚举 `directionInitTypeDef` 与 BSR 用的 `BSRExitPileTypeDef` 步骤类似，但 BSR 由 `BaseStationRecommendExitPile` 驱动，见 Q1 ③。


---



### Q3：`ExitChargePile` 各做什么？

```cpp
    void recv_out_chargepile(StampedBasicFrame_ *frame) {
        LOG_DEBUG << "COMM_CONTENT_APP_OUT_CHARGE_PILE,  " << intToHex(COMM_CONTENT_APP_OUT_CHARGE_PILE);
        RobotStatusManager::instance().notifyExitPileToEcu();
        GlobalFsm::instance().fire(MowerTrigger::START_EXIT_PILE);
    }
```



```
START_DIRECTION_INIT
```



```
ros::NodeHandle n;  // 创建句柄

// 发布话题
ros::Publisher pub = n.advertise<std_msgs::String>("chat", 10);

// 订阅话题
ros::Subscriber sub = n.subscribe("chat", 10, callback);

// 创建定时器
ros::Timer timer = n.createTimer(ros::Duration(0.1), callback);
```



## 二、方向校准状态机（MapManager 侧）

### Q3：`buildMapDirectionInitStart / Moving / Done / TurnAround` 各做什么？

```
START ──buildMapDirectionInitStart()──► MOVING
  │                                      │
  │ 未充电/GPS弱/planner失败              │ buildMapDirectionInitMoving()
  ▼                                      ▼
ERROR ◄────────────────────────────── DONE
  │                                      │
  │ buildMapDirectionError()             │ buildMapDirectionInitDone()
  ▼                                      ▼
[*]                              TURN_AROUND ──buildMapDirectionTurnAround()──► FINISHED
                                         │
                                         └── 调头失败 ──► ERROR
```

| 函数 | 关键动作 | 返回值含义 |
|------|----------|------------|
| `buildMapDirectionInitStart()` | 检查充电状态 → 感知设桩位 → `setModeSyncByAction(CALIBRATION)` | `0` 成功，`-1` 失败 |
| `buildMapDirectionInitMoving()` | 轮询 planner 进度 → `sendDirectionInitProgressToApp` | `0` 完成，`1` 继续，`<0` 失败 |
| `buildMapDirectionInitDone()` | 取桩定位 → `PLANNER_PILE_CONFIG` → 存图 → `setCalParam(1)` 允许调头 | `0` 成功，`-1` 失败 |
| `buildMapDirectionTurnAround()` | 等调头 mission 完成 → `setModeSync(IDLE)` → `setDirectionInitSucc(true)` | `0` 成功，`1` 继续，`<0` 失败 |

**注意：** 方向校准的进度走 **`sendDirectionInitProgressToApp`（0xCB）**；建图退桩走 **`sendEditDirectionInitProgressToApp`（出桩进度）**——两者不要混淆。

---

### Q4：什么情况下会进入 `DIRECTION_INIT_ERROR`？

在 `DirectionInit::directionloop()`（`mower_state.cpp`）里，除 planner 失败外，还有这些本地判定：

1. **GPS 弱**（`getGpsIntensity() <= 0`）— Start/Moving 阶段
2. **用户按 STOP**（`getPressStopDuringExitChargePile()`）
3. **蓝牙断开**（非自动建图场景会 pause，不直接报错）
4. **循环超时** — Moving/Done 超过 100 次（约 16s）
5. **MapManager 子步骤返回 -1** — 未充电、感知/planner action 失败等

进入 `DIRECTION_INIT_ERROR` 后会：

- 调用 `MapManager::buildMapDirectionError()`
- `setModeSyncByAction(PLANNER_MODE_REMOTE_CONTROL)` 并停车
- 若是自动建图：`fire(ANY_TO_IDLE)` + 云端事件 `AM_CLOUD_DATA_EVENT_TYPE_DM_EXIT_DIRECINIT_FAIL`

---

## 三、`sendDirectionErrorStateToApp` 详解

### Q5：这个函数做什么？和 `sendDirectionErrorToApp` 有什么区别？

实现位于 `bluetooth_manager.cpp`：

```cpp
int BluetoothManager::sendDirectionErrorStateToApp(uint8_t down_pile_status_byte) {
    return uploadDynamicDateToApp(
        COMM_DYNAMIC_DOWN_PILE_TASK_STATUS,  // 0x001A
        {static_cast<uint8_t>(down_pile_status_byte)});
}
```

| 函数 | 协议 | 内容 | 作用 |
|------|------|------|------|
| **`sendDirectionErrorStateToApp(byte)`** | 动态数据 `0x001A` | **1 字节失败原因** | 告诉 APP「为什么失败」 |
| **`sendDirectionErrorToApp()`** | `COMM_CONTENT_APP_LOCAL_INIT_FINISH` | 10 字节固定 `0xFF` | 告诉 APP「校准/退桩流程结束（含失败）」 |

**必须两个一起发**：先带原因，再通知结束。`map_manager.cpp` 里两处都是这样做的。

---

### Q6：`fail_reason` 从哪来？

`MapManager::buildMapDirectionError()`：

```cpp
uint8_t fail_reason = 0;
PlannerManager::instance().getPlannerMissionFailedReason(fail_reason, PLANNER_MODE_CALIBRATION);
BluetoothManager::instance().sendDirectionErrorStateToApp(fail_reason);
BluetoothManager::instance().sendDirectionErrorToApp();
```

链路：

```
PlannerManager::getPlannerMissionFailedReason()
  → runActionTask(PLANNER_GET_MISSION_COMPLETION, mode=CALIBRATION)  // cmd 114
  → result->data[1] == PLANNER_MISSION_FAILED
  → fail_reason = result->data[2]   // 具体含义在 planner 算法包，不在 mower_fsm
```

**`mower_fsm` 不定义错误码枚举**，只透传 planner 返回的第 3 字节。

若 planner 调用失败或 `data.size() < 3`，`fail_reason` 保持 **0**，日志会有 `getPlannerMissionFailedReason ... ret/size` 报错。

---

### Q7：`map_manager.cpp` 里哪些地方会调 `sendDirectionErrorStateToApp`？

全项目共 **3 处**，其中 **2 处在 map_manager**：

| 调用点 | 场景 | planner_mode | 额外动作 |
|--------|------|--------------|----------|
| `buildMapDirectionError()` | 建图方向校准失败 | `PLANNER_MODE_CALIBRATION` | 仅两个 send |
| `baseStationRecommendExitPileErrorProc()` | BSR 退桩失败 | `PLANNER_MODE_CALIBRATION` | 另发 `sendBSRExitPileError()`（0x0003） |
| `ExitChargePile::loopCheckFinish()`（`mower_state.cpp`） | 建图退桩失败 | `PLANNER_MODE_DOWN_PILE`（7） | 不在 map_manager，但协议相同 |

BSR 退桩错误处理（`map_manager.cpp`）：

```cpp
void MapManager::baseStationRecommendExitPileErrorProc() {
    uint8_t fail_reason = 0;
    PlannerManager::instance().getPlannerMissionFailedReason(fail_reason, PLANNER_MODE_CALIBRATION);
    BluetoothManager::instance().sendDirectionErrorStateToApp(fail_reason);
    BluetoothManager::instance().sendDirectionErrorToApp();
    BluetoothManager::instance().sendBSRExitPileError();
}
```

---

## 四、BSR 退桩状态机（MapManager 提供步骤函数）

### Q8：BSR 退桩和方向校准有什么关系？

**复用同一套 planner 模式 `PLANNER_MODE_CALIBRATION`**，但：

- 状态机驱动在 `BaseStationRecommendExitPile::exitPileCallback()`（`mower_state.cpp`）
- 每步调用 `MapManager` 的 `baseStationRecommendExitPile*Proc()` 系列
- 进度上报用 **`sendEditDirectionInitProgressToApp`**（和建图退桩一样），不是 `sendDirectionInitProgressToApp`

状态流转：

```
EXIT_PILE_START      → baseStationRecommendExitPileStartrProc()      → setModeSync(CALIBRATION)
EXIT_PILE_MOVING     → baseStationRecommendExitPileMovingProc()      → 轮询 moving mission
EXIT_PILE_DONE_MOVING→ baseStationRecommendExitPileDoneProc()        → setCalParam(1)
EXIT_PILE_TURN_AROUND→ baseStationRecommendExitPileTurningProc()      → 轮询 turn mission
EXIT_PILE_FINISHED   → fire(FINISH_BASE_STATION_RECOMMEND_EXIT_PILE)
EXIT_PILE_ERROR      → baseStationRecommendExitPileErrorProc()         → sendDirectionErrorStateToApp
```

任意步骤失败或 `loop_cnt_ > kExitPileLoopMaxCnt`（100）会进入 `EXIT_PILE_ERROR`。

---

## 五、排查向问答

### Q9：日志里看到 `down_pile_task(0x001A) status: X`，怎么查？

1. 往前找 **`getPlannerMissionFailedReason, mission_status: 2, failure_reason: X`**（`planner_manager.cpp`）
2. 确认是哪个 FSM 状态：`DirectionInit` / `BaseStationRecommendExitPile` / `ExitChargePile`
3. 确认 planner_mode：`CALIBRATION(101)` 还是 `DOWN_PILE(7)`
4. **X 的具体含义** → 查 planner 算法仓库，不在 `mower_fsm`

---

### Q10：方向校准失败但 `failure_reason: 0`，可能原因？

- planner 根本没进入 failed 态（本地错误：GPS 弱、未充电、感知 action 失败）— 此时可能**还没走到 planner**，`getPlannerMissionFailedReason` 取不到有效 reason
- `PLANNER_GET_MISSION_COMPLETION` 调用失败或返回 data 不够
- 建议同时看：`GPS signal is weak`、`robot not on charge pile`、`Failed! [Perception]`、`buildMapDirectionInitMoving too many times`

---

### Q11：成功和失败时 APP 分别收到什么？

| 结果 | 进度 | 错误 | 结束通知 | 成功标记 |
|------|------|------|----------|----------|
| 方向校准成功 | `sendDirectionInitProgressToApp(100)` | — | — | `setDirectionInitSucc(true)` → 后续 `sendDirectionInitSuccToApp()` |
| 方向校准失败 | 可能停在中间进度 | `0x001A` + reason | `LOCAL_INIT_FINISH` | — |
| BSR 退桩成功 | `sendEditDirectionInitProgressToApp(100)` | — | — | FSM 切下一状态 |
| BSR 退桩失败 | — | `0x001A` + reason + `BSR_EXIT_PILE_ERROR` | `LOCAL_INIT_FINISH` | — |
| 建图退桩失败 | — | `0x001A` + reason（`mower_state.cpp`） | `LOCAL_INIT_FINISH` | — |

---

### Q12：自动建图（Drop & Mow）失败有什么特殊处理？

触发链：`START_AUTO_MAP_DIRECTION_INIT` → `DirectionInit`  
失败时除了 `buildMapDirectionError()`，还会：

- `GlobalFsm::fire(ANY_TO_IDLE)`
- 云端事件 `AM_CLOUD_DATA_EVENT_TYPE_DM_EXIT_DIRECINIT_FAIL`

退桩失败（`ExitChargePile`）也有同样云端上报，但实现在 `mower_state.cpp`，不在 `map_manager.cpp`。

---

## 六、快速对照表

```
MapManager 函数                         谁驱动状态机              失败时谁调 sendDirectionErrorStateToApp
────────────────────────────────────────────────────────────────────────────────────────────
buildMapDirectionInitStart/Moving/      DirectionInit             buildMapDirectionError()
Done/TurnAround

baseStationRecommendExitPile*           BaseStationRecommend      baseStationRecommendExitPileErrorProc()
                                        ExitPile

buildMapDirectionInitProc()             （无调用，遗留）           内部 buildMapDirectionError()
```

---

## 七、关键代码索引

| 主题 | 文件 | 行号（约） |
|------|------|-----------|
| 方向校准内部状态机（遗留） | `map_manager.cpp` | 925–1016 |
| `buildMapDirectionError` | `map_manager.cpp` | 1431–1439 |
| BSR 退桩步骤与错误 | `map_manager.cpp` | 3619–3717 |
| `DirectionInit` 定时驱动 | `mower_state.cpp` | 486–669 |
| 建图退桩失败上报 | `mower_state.cpp` | 1655–1660 |
| `getPlannerMissionFailedReason` | `planner_manager.cpp` | 1370–1385 |
| `sendDirectionErrorStateToApp` | `bluetooth_manager.cpp` | 3840–3846 |
| 状态枚举定义 | `config.h` | 894–912 |

---

## 八、相关 GlobalFsm 状态与 Trigger

| GlobalFsm 状态 | 相关 Trigger | MapManager 入口 |
|----------------|--------------|-----------------|
| `DIRECTION_INIT` | `START_DIRECTION_INIT` / `FINISH_DIRECTION_INIT` | `buildMapDirectionInit*` |
| `EXIT_CHARGE_PILE` | `START_EXIT_PILE` / `FINISH_EXIT_PILE` | 退桩逻辑主要在 `mower_state.cpp` |
| `BASE_STATION_RECOMMEND_EXIT_PILE` | `START_BASE_STATION_RECOMMEND_EXIT_PILE` | `baseStationRecommendExitPile*` |

---

*文档生成自 mower_fsm 代码库分析。Planner 错误码具体枚举需参考算法侧文档。*





基站推荐和校准



```
CALIBRATION (
```



