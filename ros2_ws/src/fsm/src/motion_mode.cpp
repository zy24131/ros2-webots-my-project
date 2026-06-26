#include "fsm/motion_mode.hpp"

namespace fsm {

const char * ModeToString(RobotMode mode)
{
  switch (mode) {
    case kNarrowTrack:
      return mode_str::kNarrowTrack;
    case kSwitchToWide:
      return mode_str::kSwitchToWide;
    case kWideTrack:
      return mode_str::kWideTrack;
    case kSwitchToNarrow:
      return mode_str::kSwitchToNarrow;
    case kSpinLeft:
      return mode_str::kSpinLeft;
    case kSpinRight:
      return mode_str::kSpinRight;
    default:
      return mode_str::kUnknown;
  }
}

RobotMode ModeFromString(const std::string & mode)
{
  if (mode == mode_str::kNarrowTrack) {
    return kNarrowTrack;
  }
  if (mode == mode_str::kWideTrack) {
    return kWideTrack;
  }
  if (mode == mode_str::kSpinLeft) {
    return kSpinLeft;
  }
  if (mode == mode_str::kSpinRight) {
    return kSpinRight;
  }
  if (mode == mode_str::kSwitchToWide || mode == mode_str::kSwitchToNarrow) {
    return mode == mode_str::kSwitchToWide ? kSwitchToWide : kSwitchToNarrow;
  }
  return kRobotModeUnknown;
}

}  // namespace fsm
