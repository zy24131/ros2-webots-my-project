# 项目简介

## 一、仿 Android 的 Handler + Looper + MessageQueue 调度框架

设计并实现仿 Android 的异步消息调度框架，采用 Handler + Looper + MessageQueue 三层架构，解决割草机业务层多源异步事件（蓝牙、定时器等）与全局状态机协同中的线程调度与任务串行化问题。

- **HandlerThread**：封装后台线程生命周期，内部关联 Looper 与 MessageQueue，实现「一个后台线程 + 一个消息循环」；互斥锁 + 条件变量保证业务侧获取调度器时线程已就绪；支持设置退出标记并唤醒消费者，优雅关停。
- **MessageQueue**：按执行时间排序的消息链表，支持即时任务与延迟任务混合入队；操作队列时加锁，阻塞等待在锁外执行，避免消费者睡眠阻塞生产者；基于 **eventfd + epoll** 阻塞唤醒，仅在消费者休眠且队头变化时 wake，替代忙轮询。
- **Handler / Looper**：Handler 绑定 Looper 并共享 MessageQueue，统一投递入口；Looper 在专用线程循环 `next()` 取消息并 `dispatchMessage`，同一队列内任务严格串行。业务通过 `Runnable` 虚接口解耦，只需封装任务并 `post`，框架负责线程切换与调度。

全局状态机以即时投递为主：`post(Runnable)` → 打包 Message → 按 `mWhen` 入队 → Looper 取出后执行 `run()`。

---

## 二、基于 stateless4cpp 的 GlobalFsm

基于 `stateless4cpp` 实现全局状态机 `GlobalFsm`（单例），状态为 `MowerState`，事件为 `MowerTrigger`，在 `configStates()` 中配置 permit / onEntry / onExit 等转移规则。

- **初始化**：`init()` 中调用 `configStates()` 填配置并创建状态机引擎；同时创建名为 `GlobalStateMachine` 的 `HandlerThread` 与绑定其上的 `Handler`，后续所有状态切换都投递到该线程。
- **事件入口**：`fire(trigger[, param])` 将事件封装为 `GlobalFsmTriggerRunnable`，经 `mHandler->post()` 异步入队；在 Handler 工作线程中串行执行真正的状态迁移（`inBackThreadFire`），避免蓝牙回调、定时器等多线程直接并发改状态。
- **职责分离**：调度框架负责事件串行化与线程隔离；`stateless4cpp` 负责转移规则与 onEnter / onExit 回调。业务状态类（如 `DirectionInit`）继承 `BaseState`，在 onEnter / onExit 中落地具体流程。

```
任意线程 fire/post → MessageQueue 入队 → HandlerThread 串行 dispatch → 状态机引擎迁移
```

---

## 三、DirectionInit（方向校准）实现

`DirectionInit` 是 GlobalFsm 下的建图方向校准状态，继承 `BaseState`。进入条件包括：建图模式下 APP 发 **0x12**，或自动建图且本地无地图（`START_AUTO_MAP_DIRECTION_INIT`）。

- **外层驱动**：`onEnter` 启动 ROS 定时器，周期性调用 `directionloop()`（默认 160ms；进入调头阶段改为 50ms）；`onExit` 停止定时器并清理。
- **内层子状态**：`direction_init_state_` 按  
  `START → MOVING → DONE → TURN_AROUND → FINISHED / ERROR` 推进；每步调用 `MapManager` 对应接口（`buildMapDirectionInitStart/Moving/Done/TurnAround`），成功则进入下一步，失败或轮询超限（如 loopcnt > 100）进入 ERROR。
- **保护逻辑**：各关键步骤校验 GPS；手动校准时蓝牙断开则暂停并拉长轮询周期；按下 STOP 直接置 ERROR；失败时 `buildMapDirectionError()` 上报并切回遥控模式、停车。
- **结束跳转**：成功时，自动建图 `fire(RELOCATE_AUTO_MAP)`，手动校准切遥控并播报成功；失败且为自动建图时 `fire(ANY_TO_IDLE)` 并上报云端事件。状态切换均经 `GlobalFsm::fire` 投递到 Handler 线程执行，保证与调度框架协同安全。
