// 运动模式 / FSM 状态 / Service 意图的单一来源（SSOT）。
//
// RobotMode     — /my_robot/mode 字符串与 /my_robot/fsm_state 数值（case 0~5）
// MotionIntent  — SetMotionMode.srv（用户意图：转向 / 自转）
// TrackWidthIntent — SetTrackWidthSwitch.srv（轮距意图）
// FsmTrigger    — fsm 节点内部触发器

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

/// FSM 稳定/过渡/自转状态，与 fsm_state Int32 一致
enum RobotMode : int32_t {
  kNarrowTrack = 0,
  kSwitchToWide = 1,
  kWideTrack = 2,
  kSwitchToNarrow = 3,
  kSpinLeft = 4,
  kSpinRight = 5,
};

constexpr RobotMode kRobotModeUnknown = static_cast<RobotMode>(-1);

/// SetMotionMode.srv：0=转向 1=顺时针自转 2=逆时针自转
enum MotionIntent : int32_t {
  kSteering = 0,
  kSpinClockwise = 1,
  kSpinCounterClockwise = 2,
};

/// SetTrackWidthSwitch.srv：0=窄 1=宽（用户意图，非当前 FSM 状态）
enum TrackWidthIntent : int32_t {
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

// 兼容旧名
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

/// 切换失败或取消时回到的稳定轮距状态
inline RobotMode StableTrackBeforeSwitch(RobotMode switching)
{
  return switching == kSwitchToWide ? kNarrowTrack : kWideTrack;
}

/// 切换成功后的稳定轮距状态
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

inline bool IsValidMotionIntent(int32_t value)
{
  return value >= kSteering && value <= kSpinCounterClockwise;
}

inline bool IsValidTrackWidthIntent(int32_t value)
{
  return value == kTrackNarrow || value == kTrackWide;
}

}  // namespace fsm
