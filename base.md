

```cpp
    class BaseStationRecommendExitPile : public BaseState {
    public:
        void onEnter(SPtrTransition<MowerState,MowerTrigger> transition,SPtrParam param);
        void onExit(SPtrTransition<MowerState,MowerTrigger> transition);
    private:
        void exitPileCallback();//定时器每 500ms 调一次，BSR 退桩的 主循环
    private:
        ros::Timer exit_pile_timer_;  //ROS 周期性定时器
        std::atomic<bool> exit_pile_stop_{false}; //是否停止退桩 loop 的标志
        std::atomic<BSRExitPileTypeDef>  exit_pile_state_{BSRExitPileTypeDef::EXIT_PILE_START};
        //exitPileCallback 里 switch (exit_pile_state_) 根据它决定调哪个 MapManager 函数
        int loop_cnt_ = 0;//前步骤已循环次数
        const int kExitPileLoopMaxCnt = 100;//单步最大轮询次数
    };
```



```cpp

    void BaseStationRecommendExitPile::exitPileCallback() {
        BASE_LOG_INFO << "exitPileCallback loop time: "
                      << std::chrono::system_clock::now().time_since_epoch().count();
        if(0 == BluetoothManager::instance().getBluetoothConnect()){ //蓝牙断开，退出基站推荐状态
            BASE_LOG_WARN << "Bluetooth disconnected, exit STATE_BaseStationRecommend";
            GlobalFsm::instance().fire(MowerTrigger::EXIT_BASE_STATION_RECOMMEND);
            MapManager::instance().setStartFromChargePile(false);
            return;
        }
        if(exit_pile_stop_) {  //停止标志，退出基站推荐状态
            exit_pile_timer_.stop();
            BASE_LOG_DEBUG << "exit_pile_timer_ stopped";
            return;
        }
        int start_ret = 0, moving_ret = 0, done_moving_ret = 0, turn_ret = 0;
        switch (exit_pile_state_) {   //退出桩状态机
            case BSRExitPileTypeDef::EXIT_PILE_START: //开始退出桩流程
                start_ret = MapManager::instance().baseStationRecommendExitPileStartrProc();//开始退出桩流程
                if(0 == start_ret) {
                    exit_pile_state_.store(EXIT_PILE_MOVING);
                } else {
                    exit_pile_state_.store(EXIT_PILE_ERROR);
                }
                break;
            case BSRExitPileTypeDef::EXIT_PILE_MOVING: //移动中退出桩流程
                moving_ret = MapManager::instance().baseStationRecommendExitPileMovingProc();
                if(0 == moving_ret) {
                    exit_pile_state_.store(EXIT_PILE_DONE_MOVING);
                    loop_cnt_ = 0;
                } else if (moving_ret < 0) {
                    exit_pile_state_.store(EXIT_PILE_ERROR);
                }
                if(loop_cnt_ > kExitPileLoopMaxCnt) {
                    exit_pile_state_.store(EXIT_PILE_ERROR);
                    BASE_LOG_ERROR << "Exit pile moving too many times";
                }
                loop_cnt_++;
                break;
            case BSRExitPileTypeDef::EXIT_PILE_DONE_MOVING:// 运动完成
                done_moving_ret = MapManager::instance().baseStationRecommendExitPileDoneProc();
                if(0 == done_moving_ret) {
                    exit_pile_state_.store(EXIT_PILE_TURN_AROUND);
                    loop_cnt_ = 0;
                } else {
                    exit_pile_state_.store(EXIT_PILE_ERROR);
                }
                if(loop_cnt_ > kExitPileLoopMaxCnt) {
                    exit_pile_state_.store(EXIT_PILE_ERROR);
                    BASE_LOG_ERROR << "Exit pile moving done too many times";
                }
                loop_cnt_++;
                break;
            case BSRExitPileTypeDef::EXIT_PILE_TURN_AROUND:// 原地转向
                turn_ret = MapManager::instance().baseStationRecommendExitPileTurningProc();
                if(0 == turn_ret) {
                    exit_pile_state_.store(EXIT_PILE_FINISHED);
                } else if (turn_ret > 0) {
                    exit_pile_state_.store(EXIT_PILE_TURN_AROUND);
                    if(loop_cnt_ > kExitPileLoopMaxCnt) {
                        exit_pile_state_.store(EXIT_PILE_ERROR);
                        BASE_LOG_ERROR << "Exit pile turn around too many times";
                    }
                    loop_cnt_++;
                } else {
                    exit_pile_state_.store(EXIT_PILE_ERROR);
                }
                break;
            case BSRExitPileTypeDef::EXIT_PILE_FINISHED:// 完成退出桩流程
                BASE_LOG_INFO << "Exit pile finished";
                PlannerManager::instance().setModeSyncByAction(PLANNER_MODE_REMOTE_CONTROL);
                RobotStatusManager::instance().playVoice(AM_VOICE_DISPLAY_MODE_SUCCESS);
                // MapManager::instance().baseStationRecommendWaitProc();
                GlobalFsm::instance().fire(MowerTrigger::GO_BASE_STATION_RECOMMEND_STANDBY);
                exit_pile_stop_.store(true);
                break;
            case BSRExitPileTypeDef::EXIT_PILE_ERROR:
                BASE_LOG_ERROR << "Exit pile error";
                MapManager::instance().setStartFromChargePile(false);
                MapManager::instance().baseStationRecommendExitPileErrorProc();
                PlannerManager::instance().setModeSyncByAction(PLANNER_MODE_REMOTE_CONTROL);
                GlobalFsm::instance().fire(MowerTrigger::EXIT_BASE_STATION_RECOMMEND);
                exit_pile_stop_.store(true);
                break;
            default:
                break;
        }
    }
```

