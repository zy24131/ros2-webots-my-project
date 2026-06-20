#ifndef ROBOT_FSM__FSM_TRIGGERS_HPP_
#define ROBOT_FSM__FSM_TRIGGERS_HPP_

#include <cstdint>

namespace robot_fsm {

enum FsmState : int32_t {
  kNarrowTrack = 0,
  kSwitchToWide = 1,
  kWideTrack = 2,
  kSwitchToNarrow = 3,
  kSpinLeft = 4,
  kSpinRight = 5,
};

enum MotionModeSwitch : int32_t {
  kSteering = 0,
  kSpinClockwise = 1,
  kSpinCounterClockwise = 2,
};

enum TrackWidthSwitch : int32_t {
  kTrackNarrow = 0,
  kTrackWide = 1,
};

enum class FsmTrigger {
  kNone,
  kSpinClockwise,
  kSpinCounterClockwise,
  kReturnFromSpin,
  kRequestWide,
  kRequestNarrow,
  kActionSucceeded,
  kActionFailed,
};

const char * ModeName(FsmState state);

}  // namespace robot_fsm

#endif  // ROBOT_FSM__FSM_TRIGGERS_HPP_
