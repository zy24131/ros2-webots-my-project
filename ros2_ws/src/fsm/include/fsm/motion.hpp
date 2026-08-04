// 运动 / 轮距 命名规范（SSOT）
//
// Motion*  — 转向、顺/逆时针自转（SetMotion.srv、Motion）
// Track*   — 窄/宽轮距（SetTrack.srv、SetTrackAction.action、TrackIntent）
//
// RobotMode  — /my_robot/mode 字符串（case 0~5）
// FsmTrigger — fsm 节点内部触发器

#pragma once

#include <cstdint>
#include <string>

namespace fsm {

/// /my_robot/mode 话题字符串常量
namespace mode_str {
inline constexpr const char kNarrowTrack[] = "narrow_track";
inline constexpr const char kSwitchToWide[] = "switch_to_wide";
inline constexpr const char kWideTrack[] = "wide_track";
inline constexpr const char kSwitchToNarrow[] = "switch_to_narrow";
inline constexpr const char kSpinLeft[] = "spin_left";
inline constexpr const char kSpinRight[] = "spin_right";
inline constexpr const char kUnknown[] = "unknown";
}  // namespace mode_str

/// FSM 稳定/过渡/自转状态（case 0~5，经 /my_robot/mode 发布）
enum RobotMode : int32_t {
  kNarrowTrack = 0,
  kSwitchToWide = 1,
  kWideTrack = 2,
  kSwitchToNarrow = 3,
  kSpinLeft = 4,
  kSpinRight = 5,
};

constexpr RobotMode kRobotModeUnknown = static_cast<RobotMode>(-1);

/// SetMotion.srv：0=转向 1=顺时针自转 2=逆时针自转
enum Motion : int32_t {
  kMotionSteering = 0,
  kMotionSpinClockwise = 1,
  kMotionSpinCounterClockwise = 2,
};

constexpr Motion kSteering = kMotionSteering;
constexpr Motion kSpinClockwise = kMotionSpinClockwise;
constexpr Motion kSpinCounterClockwise = kMotionSpinCounterClockwise;

/// SetTrack.srv：0=窄 1=宽（用户意图，非当前 FSM 状态）
enum TrackIntent : int32_t {
  kTrackNarrow = 0,
  kTrackWide = 1,
};

enum class FsmTrigger {
  kNone,
  kMotionSpinClockwise,
  kMotionSpinCounterClockwise,
  kMotionReturnFromSpin,
  kTrackRequestWide,
  kTrackRequestNarrow,
};

using FsmState = RobotMode;

const char * ModeToString(RobotMode mode);
RobotMode ModeFromString(const std::string & mode);

inline const char * ModeName(RobotMode mode)
{
  return ModeToString(mode);
}

inline bool IsSwitching(RobotMode mode)
{
  return mode == kSwitchToWide || mode == kSwitchToNarrow;
}

inline RobotMode StableTrackBeforeSwitch(RobotMode switching)
{
  return switching == kSwitchToWide ? kNarrowTrack : kWideTrack;
}

inline RobotMode StableTrackAfterSwitch(RobotMode switching)
{
  return switching == kSwitchToWide ? kWideTrack : kNarrowTrack;
}

inline bool IsSpin(RobotMode mode)
{
  return mode == kSpinLeft || mode == kSpinRight;
}

inline bool IsTrackDriving(RobotMode mode)
{
  return mode == kNarrowTrack || mode == kWideTrack;
}

inline bool IsValidMotion(int32_t value)
{
  return value >= kMotionSteering && value <= kMotionSpinCounterClockwise;
}

inline bool IsValidTrackIntent(int32_t value)
{
  return value == kTrackNarrow || value == kTrackWide;
}

}  // namespace fsm
