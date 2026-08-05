```cpp
    void GlobalFsm::init() {
        lock_guard<mutex> lockGuard(_global_mutex);
        if (_init_flag) {
            LOG_INFO<<"GlobalFsm already init";
        } else {
            _init_flag = true;
            LOG_INFO<<"GlobalFsm init";
            // 填 _shared_config（permit/onEntry/onExit），末尾创建 _sptr_state_machine
            configStates();
            // fire()/setEnterInit() 都 post 到此线程，保证状态切换串行
            mHandlerThread = std::make_shared<HandlerThread>("GlobalStateMachine");
            mHandlerThread->start();
            mHandler = std::make_shared<Handler>(mHandlerThread->getLooper());
        }
    }
```



```cpp
    void HandlerThread::start() {
#ifdef DEBUG
        printf("HandlerThread %s start\n",mName.c_str());
#endif

        mThread = make_shared<thread>(&HandlerThread::run,this);
        {
            lock_guard<mutex> lock(mMutex);
            mState = RUNNING;
        }
        mCv.notify_all();
    }
```

`start()` — 创建并启动线程，把 `run` 当作线程入口；`run()` = 那条线程里干的事（跑 Looper 循环取任务）。 不是把 run「塞进队列」

```cpp
    void HandlerThread::run() {
        mTid = this_thread::get_id();
#ifdef DEBUG
        cout<<"HandlerThread["<<mName<<"]:run tid "<<mTid<<endl;
#endif
        mLooper->loop();
    }

```



```cpp
    // 消息循环：在 HandlerThread 工作线程里死循环，从队列取任务并交给 Handler 执行
    void Looper::loop() {
        if(!mQueue) return;
        for(;;){
            // 阻塞等待队头 Message；队列 quit 时返回 nullptr，退出 loop
            SptrMessage msg = mQueue->next();
            if(msg){
                if(msg->mTarget){
                    // mTarget 即 post 时的 Handler；内部执行 Runnable（如 GlobalFsm::fire）
                    msg->mTarget->dispatchMessage(msg);
                    msg.reset();
                }else{
                    cout<<"msg target is null"<<endl;
                }
            }else{
                return;  // MessageQueue::quit() 后 next() 返回空，线程 loop 结束
            }
        }
    }
```



```cpp
class Message {
    std::time_t mWhen;              // 执行时间（支持延迟 post）
    std::shared_ptr<Runnable> mCallback;  // 要跑的任务（如 fire）
    SptrHandler mTarget;            // 哪个 Handler 投递的
    Message* mNext;                 // 链表下一项
};
```



```cpp
class Runnable{
    public:
        virtual void run() = 0;
        virtual ~Runnable() = default;
    };
```



```cpp
post( Runnable )
       │
       ▼ 包一层
   Message { mCallback=Runnable, mTarget=Handler, mWhen=时间 }
       │
       ▼ 入队
   MessageQueue._messages  ──→  Message ──→ Message ──→ ...
       │
       ▼ next()
   Looper::loop 取队头
       │
       ▼
   mCallback->run()
```

1.全局状态机

```cpp
class GlobalFsm {
    public:
        static GlobalFsm &instance() {
            static GlobalFsm instance;
            return instance;
        }
        GlobalFsm(GlobalFsm const &) = delete;
        void operator=(GlobalFsm const &) = delete;
        void fire(MowerTrigger trigger);
        void fire(MowerTrigger trigger, stateless4cpp::SPtrParam sPtrParam);
    private:
        GlobalFsm();
    private:
        stateless4cpp::SPtrStateMachineConfig<MowerState, MowerTrigger> _shared_config; 
        bs_tools::SptrHandler mHandler;
    };
```



```cpp
    class Runnable{
    public:
        virtual void run() = 0;
        virtual ~Runnable() = default;
    };
```

```cpp
    class GlobalFsmTriggerRunnable : public bs_tools::Runnable {
    public:
        GlobalFsmTriggerRunnable(MowerTrigger trigger, stateless4cpp::SPtrParam param):
            mTrigger(trigger),
            mParam(param){}

        void run();

    private:
        MowerTrigger mTrigger;
        SPtrParam mParam;
    };
```



```cpp
    class Message{
    public:
        Message();
        ~Message();

    public:
        std::time_t mWhen;
        int mWhat;
        std::shared_ptr<Runnable> mCallback;
        static SptrMessage obtain(SptrHandler handler,int what);
        static SptrMessage obtain();
        SptrHandler mTarget;
    private:
        std::shared_ptr<Message> mNext;

        friend class Handler;
        friend class MessageQueue;
    };
```

```
post(runnable)
  → getPostMessage()      // 只是打包，还没进队
  → sendMessage(msg)
  → sendMessageDelayed(msg, 0)
  → sendMessageAtTime(msg, now)
  → mQueue->enqueueMessage(msg, when)   // ← 这里才真正进 MessageQueue
```



```cpp
    //全局状态机的「发事件切状态」入口。
	void GlobalFsm::fire(MowerTrigger trigger, stateless4cpp::SPtrParam sPtrParam) {
        //把 {trigger, param} 包成 GlobalFsmTriggerRunnable
        mHandler->post(make_shared<GlobalFsmTriggerRunnable>(trigger, sPtrParam));
    }


    // 把 Runnable 包装成 Message 并入队；返回 true 仅表示 enqueue 成功，run() 尚未执行
    bool Handler::post(SptrRunnable runnable) {
        SptrMessage msg = getPostMessage(runnable);
        return sendMessage(msg);
    }

    // 立即发送（delay = 0）
    bool Handler::sendMessage(SptrMessage msg) {
        return sendMessageDelayed(msg,0);
    }

    // 延迟 delayMillis 毫秒后入队
    bool Handler::sendMessageDelayed(SptrMessage msg, long delayMillis) {
        if(delayMillis < 0){
            delayMillis =  0;
        }
        return sendMessageAtTime(msg,getCurrentTimeStamp()+delayMillis);
    }

    // 在指定时刻入队（最终入口）；uptimeMillis 为消息最早被 Looper 取出的时间
    bool Handler::sendMessageAtTime(SptrMessage msg, std::time_t uptimeMillis) {
        return mQueue->enqueueMessage(msg,uptimeMillis);
    }

----------------------------------------------------------------------------------------

	//它是Handler的一个持有函数
    SptrMessage Handler::getPostMessage(SptrRunnable r) {
        SptrMessage m = Message::obtain();   // 拿一个空 Message（对象池复用）
        m->mCallback = r;                    // 任务本体，例如 GlobalFsmTriggerRunnable
        m->mTarget = shared_from_this();     // 把handler自己复制给Message
        return m;
    }


	//新建一个Message
    SptrMessage Message::obtain() {     
        return std::make_shared<Message>(); 
    }
```



```cpp
/*
 * 入队：按 when 插入 _messages 有序链表。
 * 配对 next()：Looper 阻塞取消息；必要时 wake() 打断等待。
 * post() 时 when≈now → 入队后由 Handler 线程 dispatchMessage → run()。
 */
bool MessageQueue::enqueueMessage(std::shared_ptr<Message> message, std::time_t when) {
    if (!message) return false;

    lock_guard<mutex> lock(_messages_cv_m);
    if (_quit_flag) return false;          // HandlerThread 已 quit，拒收

    message->mWhen = when;                 // 最早执行时刻；未到点 next() 会继续等

    if (!_messages || message->mWhen < _messages->mWhen) {
        message->mNext = _messages;          // 插队头
        _messages = message;
        if (mBlocked) wake();              // Looper 阻塞中 → 唤醒 next()
    } else {
        // ... 按 mWhen 升序插到链表中间/尾部
    }
    return true;                           // 仅入队成功，run() 尚未执行
}
```



```cpp
    void GlobalFsm::init() {
        lock_guard<mutex> lockGuard(_global_mutex);
        if (_init_flag) {
            LOG_INFO<<"GlobalFsm already init";
        } else {
            _init_flag = true;
            LOG_INFO<<"GlobalFsm init";
            // 填 _shared_config（permit/onEntry/onExit），末尾创建 _sptr_state_machine
            configStates();
            // fire()/setEnterInit() 都 post 到此线程，保证状态切换串行
            mHandlerThread = std::make_shared<HandlerThread>("GlobalStateMachine");
            mHandlerThread->start();
            mHandler = std::make_shared<Handler>(mHandlerThread->getLooper());
        }
    }
```



```cpp
void HandlerThread::start() {
    mThread = make_shared<thread>(&HandlerThread::run, this);  // ① 新线程开始跑 run()
    mState = RUNNING;
    mCv.notify_all();   // ② 唤醒 getLooper() 里在等的代码
}

void HandlerThread::run() {
    mTid = this_thread::get_id();
    mLooper->loop();    // ③ 进入死循环取消息
}

void Looper::loop() {
    for (;;) {
        msg = mQueue->next();           // ④ 阻塞出队
        if (!msg) return;               // quit → 线程结束

        msg->mTarget->dispatchMessage(msg);  // ⑤ run() / handleMessage
        msg.reset();
    }
}
```

```
主线程                          HandlerThread 工作线程
   │                                    │
init()                               │
   ├─ new HandlerThread               │
   ├─ start() ──────────────────────→ run()
   ├─ new Handler(looper)             │   loop()
   │                                  │     next() 阻塞等
fire/post (之后任意线程)               │
   └─ enqueueMessage ──写入队列──→   │     next() 醒来
                                      │     dispatchMessage
                                      │     run() → inBackThreadFire
                                      │     next() 继续...
```

