#ifndef ORIGINCAR_COMPITION__MISSION_TYPES_HPP_
#define ORIGINCAR_COMPITION__MISSION_TYPES_HPP_

// ============================================================================
// 任务状态机公共类型定义 —— 集中管理状态枚举与路点结构, 便于维护。
// ============================================================================

namespace origincar_compition
{

/// 任务状态机的状态。
///   INIT                 : 初始化 (等 nav2 → 发 /initialpose → 等 AMCL 收敛)
///   STARTUP_ESCAPE       : 发车脱困, 低速直行驶离起点致命栅格, 再交还 Nav2
///   GO_TO_TASK_POINT     : 前往任务发布点, 途中监控 QR + 进度
///   REORIENT             : 扫码后硬编码倒车右转, 闭环把车头复位到起点朝向
///   GO_TO_YELLOW_CHANNEL : 复位后 Nav2 前往黄色通道入口 (到达朝向=起点朝向)
///   OUTBOUND_DONE        : 已到黄色入口, 发布 ~/phase_done, 等待 ~/start_inbound
///   GO_TO_START          : 收到入站指令 → 发 initialpose(入口, 背对起点) → 等 AMCL → Nav2 回 P 点
///   DONE                 : 任务完成 (已回到 P 点)
///   FAILED               : 导航失败/中止
enum class MissionState
{
  INIT = 0,
  STARTUP_ESCAPE,
  GO_TO_TASK_POINT,
  REORIENT,
  GO_TO_YELLOW_CHANNEL,
  OUTBOUND_DONE,
  GO_TO_START,
  DONE,
  FAILED
};

/// 地图系 (map) 路点: 平面位置 + 偏航角。
struct Waypoint
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};   // rad
};

/// 状态转字符串 (日志用)。
inline const char * toString(MissionState s)
{
  switch (s) {
    case MissionState::INIT:                 return "INIT";
    case MissionState::STARTUP_ESCAPE:       return "STARTUP_ESCAPE";
    case MissionState::GO_TO_TASK_POINT:     return "GO_TO_TASK_POINT";
    case MissionState::REORIENT:             return "REORIENT";
    case MissionState::GO_TO_YELLOW_CHANNEL: return "GO_TO_YELLOW_CHANNEL";
    case MissionState::OUTBOUND_DONE:        return "OUTBOUND_DONE";
    case MissionState::GO_TO_START:          return "GO_TO_START";
    case MissionState::DONE:                 return "DONE";
    case MissionState::FAILED:               return "FAILED";
    default:                                 return "UNKNOWN";
  }
}

}  // namespace origincar_compition

#endif  // ORIGINCAR_COMPITION__MISSION_TYPES_HPP_
