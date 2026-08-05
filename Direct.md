```cpp
    class BaseState{
    public:
        BaseState() = default;
        virtual ~BaseState() = default;
        virtual void onEnter(stateless4cpp::SPtrTransition<MowerState,MowerTrigger> transition,stateless4cpp::SPtrParam param) = 0;
        virtual void onExit(stateless4cpp::SPtrTransition<MowerState,MowerTrigger> transition) = 0;
    };
```



```cpp
    class DirectionInit : public BaseState{
    public:
        void onEnter(SPtrTransition<MowerState,MowerTrigger> transition,SPtrParam param);
        void onExit(SPtrTransition<MowerState,MowerTrigger> transition);
    private:
        void directionloop(); //定时器周期性调用（默认 160ms，调头阶段改为 50ms）
        bool checkGPS();//GPS 信号是否可用
    private:
        MowerTrigger trigger_;  //进入校准时用的 trigger
        ros::Timer direction_timer_;  //ROS 周期性定时器
        directionInitTypeDef direction_init_state_;  //校准内部小状态
        std::atomic_bool loopquitflag_;//是否结束 directionloop
        int loopcnt_;//前步骤已循环次数
    };
```



```cpp
void DirectionInit::directionloop(){
        int build_init_result = 0;     //初始化结果，用于判断是否需要继续执行后续操作
        int build_moving_result = 0;   //移动结果，用于判断是否需要继续执行后续操作
        int build_done_result = 0;     //完成结果，用于判断是否需要继续执行后续操作
        int build_turn_around = 0;     //转向结果，用于判断是否需要继续执行后续操作
        LOG_INFO << "direction loop time: "
                 << std::chrono::system_clock::now().time_since_epoch().count();
        if (0 == BluetoothManager::instance().getBluetoothConnect() &&
            trigger_ !=  MowerTrigger::START_AUTO_MAP_DIRECTION_INIT) {//蓝牙断开，且不是自动建图初始化
            MapManager::instance().buildMapDirectionInitPauseProc(); //暂停建图初始化
            direction_timer_.setPeriod(ros::Duration(1), true);//暂停建图初始化后，循环时间改为1s
            return;
        }
        if (loopquitflag_.load()) { //如果已经退出循环，则直接返回
            direction_timer_.stop();//停止定时器，退出循环
            return;
        }

        if(RobotStatusManager::instance().getPressStopDuringExitChargePile()) { //如果按下停止键，则退出循环
            LOG_ERROR << "[Failed!] press stop during the exit charge pile.";
            direction_init_state_ = DIRECTION_INIT_ERROR; //设置状态为错误，退出循环
        }

        switch (direction_init_state_) { //根据状态执行不同的操作
            case DIRECTION_INIT_FINISHED: //如果状态为完成，则执行以下操作
                LOG_INFO << "direction init finished";
                if (trigger_ ==  MowerTrigger::START_AUTO_MAP_DIRECTION_INIT) { //自动建图模式
                    GlobalFsm::instance().fire(MowerTrigger::RELOCATE_AUTO_MAP); //不在充电桩上收到建图命令 ，重定位状态 
                } else { //普通模式
                    MapManager::instance().buildMapNoneProc(); //否则，执行无操作
                    PlannerManager::instance().setModeSyncByAction( //设置规划器模式为远程控制
                        PLANNER_MODE_REMOTE_CONTROL);
                    RobotStatusManager::instance().playVoice(
                        AM_VOICE_DISPLAY_MODE_SUCCESS);
                }
                loopquitflag_.store(true);//退出循环
                break;
            case DIRECTION_INIT_ERROR:
                LOG_ERROR << "direction init failed";
                MapManager::instance().buildMapDirectionError(); //执行建图初始化错误处理
                
                PlannerManager::instance().setModeSyncByAction( //设置规划器模式为远程控制
                    PLANNER_MODE_REMOTE_CONTROL);
                RemoteControlManager::instance().publishSpeed(0.0, 0.0); //发布速度为0，停止运动
                loopquitflag_.store(true);  //退出循环
                if (trigger_ ==  MowerTrigger::START_AUTO_MAP_DIRECTION_INIT) {//如果是自动建图初始化，则触发自动建图失败事件
                    DROP_LOG_INFO << "Auto map failed, stop auto map.";
                    GlobalFsm::instance().fire(MowerTrigger::ANY_TO_IDLE);//触发进入空闲状态
                    IoVManager::instance().upLoadEvent(
                        RobotStatusManager::instance().getMowerStatus(),
                        AM_CLOUD_DATA_EVENT_TYPE_DROP_MOW,
                        AM_CLOUD_DATA_EVENT_TYPE_DM_EXIT_DIRECINIT_FAIL, false);
                }
                break;
            case DIRECTION_INIT_START:
                if (!checkGPS()) {
                    direction_init_state_ = DIRECTION_INIT_ERROR;//如果GPS信号弱，则设置状态为错误，退出循环
                    LOG_WARN << "GPS signal is weak, buildMapDirectionInitStart before"
                                "failed";
                    build_init_result = -1;//设置初始化结果为-1，表示失败
                } else {
                    build_init_result =
                        MapManager::instance().buildMapDirectionInitStart();//执行建图初始化开始操作，返回结果
                }

                if (0 == build_init_result) {
                    direction_init_state_ = DIRECTION_INIT_MOVING; //如果初始化成功，则设置状态为移动中
                } else {
                    direction_init_state_ = DIRECTION_INIT_ERROR; //如果初始化失败，则设置状态为错误，退出循环
                }
                if (!checkGPS()) { //双重保险，确保数据从开始到结束都可信
                    direction_init_state_ = DIRECTION_INIT_ERROR;//如果GPS信号弱，则设置状态为错误，退出循环
                    LOG_WARN << "GPS signal is weak, buildMapDirectionInitStart end"
                                "failed";
                }
                break;
            /* init ing  */
            case DIRECTION_INIT_MOVING:   

                if (!checkGPS()) {
                    direction_init_state_ = DIRECTION_INIT_ERROR;
                    LOG_WARN << "GPS signal is weak, buildMapDirectionInitMoving "
                                "failed";
                    build_moving_result = -1;
                } else {
                    build_moving_result =
                        MapManager::instance().buildMapDirectionInitMoving();//执行建图初始化移动操作，返回结果
                }
                LOG_INFO << "build_moving_result: " << build_moving_result
                         << " loopcnt " << loopcnt_;
                if (0 == build_moving_result) {
                    direction_init_state_ = DIRECTION_INIT_DONE; //如果移动成功，则设置状态为完成
                    loopcnt_ = 0;
                    break;
                } else if (build_moving_result < 0) {
                    direction_init_state_ = DIRECTION_INIT_ERROR;//如果移动失败，则设置状态为错误，退出循环
                }
                if (loopcnt_ > 100) {
                    direction_init_state_ = DIRECTION_INIT_ERROR;//如果循环次数超过100次，则设置状态为错误，退出循环
                    LOG_WARN << "buildMapDirectionInitMoving too many times, "
                                "but not response correctly";
                }
                loopcnt_++;
                break;
            /*  init done  */
            case DIRECTION_INIT_DONE:  //如果状态为完成，则执行以下操作
                build_done_result =
                    MapManager::instance().buildMapDirectionInitDone(); //执行建图初始化完成操作，返回结果
                if (0 == build_done_result) {
                    direction_init_state_ = DIRECTION_TURN_AROUND; //如果完成成功，则设置状态为转向
                    IoVManager::instance().upLoadChargePilePosition();
                    direction_timer_.setPeriod(ros::Duration(0.05), true);
                } else if (-1 == build_done_result) {
                    direction_init_state_ = DIRECTION_INIT_ERROR;
                }
                if (loopcnt_ > 100) {
                    direction_init_state_ = DIRECTION_INIT_ERROR;
                    LOG_WARN << "buildMapDirectionInitDone too many times, but "
                                "not response correctly";
                }
                loopcnt_++;
                LOG_INFO << "build_done_result: " << build_done_result;
                break;
            case DIRECTION_TURN_AROUND://如果状态为转向，则执行以下操作

                build_turn_around =
                    MapManager::instance().buildMapDirectionTurnAround(); //执行建图初始化转向操作，返回结果
                LOG_INFO << "build_turn_around: " << build_turn_around;
                if (0 == build_turn_around) {
                    direction_init_state_ = DIRECTION_INIT_FINISHED; //如果转向成功，则设置状态为完成
                } else if (build_turn_around > 0) {
                    direction_init_state_ = DIRECTION_TURN_AROUND;  //如果转向失败，则继续转向
                } else {
                    direction_init_state_ = DIRECTION_INIT_ERROR;   //如果转向失败，则设置状态为错误，退出循环
                }

                break;
            default:
                break;
        }
    }
```

