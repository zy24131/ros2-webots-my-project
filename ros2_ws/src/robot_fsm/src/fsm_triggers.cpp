#include "robot_fsm/fsm_triggers.hpp"

namespace robot_fsm {

const char * ModeName(FsmState state)
{
  switch (state) {
    case kNarrowTrack: return "narrow_track";
    case kSwitchToWide: return "switch_to_wide";
    case kWideTrack: return "wide_track";
    case kSwitchToNarrow: return "switch_to_narrow";
    case kSpinLeft: return "spin_left";
    case kSpinRight: return "spin_right";
    default: return "unknown";
  }
}

}  // namespace robot_fsm
