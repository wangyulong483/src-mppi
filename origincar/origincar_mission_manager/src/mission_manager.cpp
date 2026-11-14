// ============================================================================
// origincar_mission_manager — 顶层任务调度者实现 (合并段1+段3版, 预热发车)
//
// 发车方式:
//   1) ros2 launch origincar_mission_manager mission.launch.py   # 预热
//   2) ros2 service call /mission_manager/start std_srvs/srv/Trigger "{}"  # 发车
//
// 流程:
//   INIT → WAITING_START (预检完成、零速等待 /mission_manager/start)
//        → SEG1_3_OUT (调 mission_state_machine ~/start, 出走 P→QR→entrance)
//        → TRANSITION (零速制动)
//        → SEG2 (放行 stage2, 惯导环道)
//        → SEG1_3_IN (调 mission_state_machine ~/start_inbound, 归程 entrance→P)
//        → FINISHED
//
// 子状态机接口:
//   段1+3 (mission_state_machine): ~/start / ~/ready / ~/phase_done / ~/start_inbound / ~/done
//   段2 (stage2_state_machine):   /stage2_state_machine/enable
//                                 /path_follower/finished
// ============================================================================

#include "origincar_mission_manager/mission_manager.hpp"

#include <cmath>
#include <functional>

namespace origincar_mission_manager
{

using std::placeholders::_1;
using std::placeholders::_2;

const char * to_string(MissionPhase p)
{
  switch (p) {
    case MissionPhase::INIT:          return "INIT";
    case MissionPhase::WAITING_START: return "WAITING_START";
    case MissionPhase::SEG1_3_OUT:    return "SEG1_3_OUT";
    case MissionPhase::TRANSITION:    return "TRANSITION";
    case MissionPhase::SEG2:          return "SEG2";
    case MissionPhase::SEG1_3_IN:     return "SEG1_3_IN";
    case MissionPhase::FINISHED:      return "FINISHED";
  }
  return "UNKNOWN";
}

MissionManager::MissionManager()
: rclcpp::Node("mission_manager")
{
  // ── 参数 ──────────────────────────────────────────────────
  skip_seg2_ = declare_parameter<bool>("skip_seg2", false);
  transition_duration_ = declare_parameter<double>("transition_duration", 0.5);
  amcl_settle_sec_ = declare_parameter<double>("amcl_settle_sec", 2.0);
  control_rate_ = declare_parameter<double>("control_rate", 10.0);
  seg2_amcl_pause_delay_ = declare_parameter<double>("seg2_amcl_pause_delay", 1.5);
  cone_enable_service_ = declare_parameter<std::string>(
    "cone_enable_service", "/lidar_cone_detector/enable");
  vlm_enable_service_ = declare_parameter<std::string>(
    "vlm_enable_service", "/vlm_node/enable");
  qr_enable_service_ = declare_parameter<std::string>(
    "qr_enable_service", "/qr_decoder_zbar/enable");

  // ── 段1+3 接口 (mission_state_machine) ────────────────────
  seg1_3_start_client_ = create_client<std_srvs::srv::Trigger>(
    "/mission_state_machine/start");
  seg1_3_inbound_client_ = create_client<std_srvs::srv::Trigger>(
    "/mission_state_machine/start_inbound");

  rclcpp::QoS done_qos(1);
  done_qos.transient_local();
  seg1_3_phase_done_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/mission_state_machine/phase_done", done_qos,
    std::bind(&MissionManager::seg1_3OutDoneCallback, this, _1));
  seg1_3_done_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/mission_state_machine/done", done_qos,
    std::bind(&MissionManager::seg1_3DoneCallback, this, _1));
  seg1_3_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/mission_state_machine/ready", done_qos,
    std::bind(&MissionManager::seg1_3ReadyCallback, this, _1));

  // ── 段2 接口 (stage2_state_machine 门控放行) ──────────────
  seg2_start_client_ = create_client<std_srvs::srv::Trigger>(
    "/stage2_state_machine/enable");
  seg2_done_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/path_follower/finished", done_qos,
    std::bind(&MissionManager::seg2DoneCallback, this, _1));

  // ── cmd_vel (WAITING_START / TRANSITION 阶段发布零速) ──────
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  ready_pub_ = create_publisher<std_msgs::msg::Bool>("~/ready", done_qos);
  start_command_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/start", std::bind(&MissionManager::onStartCommand, this, _1, _2));
  publishReady(false);

  // ── QR 扫码门控 (段1出站才开, 其余段关省CPU) ─────────
  qr_enable_client_ = create_client<std_srvs::srv::SetBool>(qr_enable_service_);

  // ── 锥桶检测门控 (段2才开) ───────────────────────────────
  cone_enable_client_ = create_client<std_srvs::srv::SetBool>(cone_enable_service_);

  // ── VLM 门控 (段2才开, 省API费用) ────────────────────────
  vlm_enable_client_ = create_client<std_srvs::srv::SetBool>(vlm_enable_service_);

  // ── 主循环 ────────────────────────────────────────────────
  const double period = 1.0 / std::max(1.0, control_rate_);
  timer_ = create_wall_timer(
    std::chrono::duration<double>(period),
    std::bind(&MissionManager::controlLoop, this));

  phase_enter_time_ = now();

  RCLCPP_INFO(get_logger(),
    "MissionManager 就绪 (预热发车): skip_seg2=%s transition=%.1fs rate=%.0fHz "
    "→ 等待 ros2 service call /mission_manager/start std_srvs/srv/Trigger",
    skip_seg2_ ? "YES" : "no", transition_duration_, control_rate_);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 状态切换
// ═══════════════════════════════════════════════════════════════════════════════

void MissionManager::transitionTo(MissionPhase next)
{
  if (next == phase_) { return; }
  RCLCPP_INFO(get_logger(), "[MM] 阶段: %s -> %s",
    to_string(phase_), to_string(next));
  phase_ = next;
  phase_enter_time_ = now();
}

double MissionManager::elapsedInPhase() const
{
  return (now() - phase_enter_time_).seconds();
}

// ═══════════════════════════════════════════════════════════════════════════════
// 主循环
// ═══════════════════════════════════════════════════════════════════════════════

void MissionManager::controlLoop()
{
  switch (phase_) {
    case MissionPhase::INIT:          phaseInit();          break;
    case MissionPhase::WAITING_START: phaseWaitingStart();  break;
    case MissionPhase::SEG1_3_OUT:    phaseSeg1_3Out();     break;
    case MissionPhase::TRANSITION:    phaseTransition();     break;
    case MissionPhase::SEG2:          phaseSeg2();           break;
    case MissionPhase::SEG1_3_IN:     phaseSeg1_3In();       break;
    case MissionPhase::FINISHED:      phaseFinished();       break;
  }
}

// ── INIT: 等所有接口上线 → 等待预检完成 → WAITING_START ────
void MissionManager::phaseInit()
{
  // 预热期间持续零速，发车指令到达前车辆保持静止
  publishZeroTwist();
  bool ready = true;

  if (!seg1_3_start_client_->service_is_ready()) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
      "[MM] 等待 mission_state_machine ~/start 服务...");
    ready = false;
  }
  if (!seg1_3_inbound_client_->service_is_ready()) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
      "[MM] 等待 mission_state_machine ~/start_inbound 服务...");
    ready = false;
  }
  if (!skip_seg2_ && !seg2_start_client_->service_is_ready()) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
      "[MM] 等待 stage2_state_machine/enable 服务...");
    ready = false;
  }
  if (!seg1_3_ready_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
      "[MM] 等待段1 Nav2/AMCL 发车预检完成...");
    ready = false;
  }
  if (!cone_enable_client_->service_is_ready() ||
      !vlm_enable_client_->service_is_ready() ||
      !qr_enable_client_->service_is_ready())
  {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
      "[MM] 等待 QR/锥桶/VLM 管理接口就绪...");
    ready = false;
  }

  if (ready) {
    setConeDetector(false);   // 段1 Nav2 不需要锥桶检测
    setVlmEnabled(false);     // 段1 不需要 VLM (省API费用)
    if (!qr_activation_requested_) {
      qr_activation_requested_ = true;
      activateQrNodes();      // 发车前完成 QR 激活，收到指令即可立即移动
    }
    if (qr_active_) {
      publishReady(true);
      transitionTo(MissionPhase::WAITING_START);
      RCLCPP_INFO(get_logger(),
        "[MM] 预热完成，车辆保持零速。请发车: "
        "ros2 service call /mission_manager/start std_srvs/srv/Trigger \"{}\"");
    }
  }
}

// ── WAITING_START: 所有预检完成，保持零速，等待发车指令 ──
void MissionManager::phaseWaitingStart()
{
  publishZeroTwist();
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
    "[MM] 等待发车指令: ros2 service call /mission_manager/start std_srvs/srv/Trigger \"{}\"");
}

// ── SEG1_3_OUT: 等 mission_state_machine 到达黄色入口 ───────
void MissionManager::phaseSeg1_3Out()
{
  if (seg1_3_out_done_) {
    RCLCPP_INFO(get_logger(), "[MM] 出站完成 → 关闭 QR 扫码, 进入过渡 (零速制动 %.1fs)",
      transition_duration_);
    deactivateQrNodes();      // 段1完成, QR 不需要了, 省CPU
    for (int i = 0; i < 10; ++i) { publishZeroTwist(); }
    transitionTo(MissionPhase::TRANSITION);
  }
}

// ── TRANSITION: 零速制动 → 启动段2 ─
void MissionManager::phaseTransition()
{
  publishZeroTwist();
  if (elapsedInPhase() >= transition_duration_) {
    if (skip_seg2_) {
      RCLCPP_INFO(get_logger(),
        "[MM] 过渡完成 → skip_seg2=true 跳过段2, 直接段1+3入站 (黄色入口→P)");
      for (int i = 0; i < 5; ++i) { publishZeroTwist(); }
      transitionTo(MissionPhase::SEG1_3_IN);
    } else {
      RCLCPP_INFO(get_logger(),
        "[MM] 过渡完成 → 启动段2 惯导环道 (AMCL 保持 active, 持续发布 map→odom TF)");
      setConeDetector(true);    // 段2 惯导需要锥桶检测
      setVlmEnabled(true);      // 段2 环道启用 VLM 图生文
      seg2SendStart();          // 放行 stage2 → path_follower 发车
      seg2_amcl_pause_done_ = false;
      transitionTo(MissionPhase::SEG2);
    }
  }
}

// ── SEG2: 等 path_follower finished; AMCL 全程保持 active ─────────
void MissionManager::phaseSeg2()
{
  if (!seg2_amcl_pause_done_ && elapsedInPhase() >= seg2_amcl_pause_delay_) {
    RCLCPP_INFO(get_logger(),
      "[MM] 段2锚定窗口结束(%.1fs) — path_follower 锚定已完成", seg2_amcl_pause_delay_);
    seg2_amcl_pause_done_ = true;
  }

  if (seg2_done_ && !seg2_pending_) {
    seg2_pending_ = true;
    RCLCPP_INFO(get_logger(), "[MM] 段2完成 → 启动段1+3 入站 (黄色入口→P)");
    setConeDetector(false);   // 段3 Nav2 不需要锥桶检测
    for (int i = 0; i < 5; ++i) { publishZeroTwist(); }
    transitionTo(MissionPhase::SEG1_3_IN);
  }
}

// ── SEG1_3_IN: 调 mission_state_machine ~/start_inbound ─
void MissionManager::phaseSeg1_3In()
{
  if (seg1_3_inbound_client_->service_is_ready()) {
    seg1_3SendInbound();
  }

  if (seg1_3_done_) {
    RCLCPP_INFO(get_logger(), "[MM] 段1+3 入站完成 → 全部任务结束!");
    transitionTo(MissionPhase::FINISHED);
  }
}

// ── FINISHED ────────────────────────────────────────────────
void MissionManager::phaseFinished()
{
  static bool reported = false;
  if (!reported) {
    reported = true;
    publishZeroTwist();
    RCLCPP_INFO(get_logger(), "🎉 全部任务完成!");
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 发车指令: /mission_manager/start → 仅在 WAITING_START 状态有效
// ═══════════════════════════════════════════════════════════════════════════════

void MissionManager::onStartCommand(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
{
  if (phase_ != MissionPhase::WAITING_START || !seg1_3_ready_) {
    resp->success = false;
    resp->message = "departure rejected: system is not fully ready or mission already started";
    RCLCPP_WARN(get_logger(), "[MM] 发车指令被拒绝：系统尚未完全就绪或已经发车");
    return;
  }
  if (start_command_received_) {
    resp->success = false;
    resp->message = "departure command was already accepted";
    return;
  }

  start_command_received_ = true;
  publishReady(false);
  transitionTo(MissionPhase::SEG1_3_OUT);
  seg1_3SendStart();
  resp->success = true;
  resp->message = "departure command accepted; segment 1 moving now";
  RCLCPP_INFO(get_logger(), "[MM] ⚡ 发车指令已确认，段1立即移动");
}

void MissionManager::publishReady(bool ready)
{
  std_msgs::msg::Bool msg;
  msg.data = ready;
  ready_pub_->publish(msg);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 接口调用
// ═══════════════════════════════════════════════════════════════════════════════

void MissionManager::seg1_3SendStart()
{
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  seg1_3_start_client_->async_send_request(
    req,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      auto resp = future.get();
      if (resp->success) {
        RCLCPP_INFO(get_logger(), "[MM] 段1+3 出站已启动: %s", resp->message.c_str());
      } else {
        RCLCPP_ERROR(get_logger(), "[MM] 段1+3 出站启动失败: %s", resp->message.c_str());
      }
    });
}

void MissionManager::seg1_3SendInbound()
{
  static bool sent = false;
  if (sent) { return; }
  sent = true;

  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  seg1_3_inbound_client_->async_send_request(
    req,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      auto resp = future.get();
      if (resp->success) {
        RCLCPP_INFO(get_logger(), "[MM] 段1+3 入站已启动: %s", resp->message.c_str());
      } else {
        RCLCPP_ERROR(get_logger(), "[MM] 段1+3 入站启动失败: %s", resp->message.c_str());
      }
    });
}

void MissionManager::seg2SendStart()
{
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  seg2_start_client_->async_send_request(
    req,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      auto resp = future.get();
      if (resp->success) {
        RCLCPP_INFO(get_logger(), "[MM] seg2 stage2 已放行: %s",
          resp->message.c_str());
      } else {
        RCLCPP_ERROR(get_logger(), "[MM] seg2 放行失败: %s",
          resp->message.c_str());
      }
    });
}

void MissionManager::setConeDetector(bool on)
{
  static bool last_state = true;
  if (on == last_state) { return; }
  last_state = on;
  if (!cone_enable_client_->service_is_ready()) { return; }
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = on;
  cone_enable_client_->async_send_request(
    req, [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture) {});
  RCLCPP_INFO(get_logger(), "[MM] 锥桶检测: %s", on ? "开启" : "关闭");
}

void MissionManager::setVlmEnabled(bool on)
{
  static bool last_state = false;
  if (on == last_state) { return; }
  last_state = on;
  if (!vlm_enable_client_->service_is_ready()) {
    if (on) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "[MM] VLM 服务尚未就绪, 稍后重试...");
    }
    return;
  }
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = on;
  vlm_enable_client_->async_send_request(
    req, [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture) {});
  RCLCPP_INFO(get_logger(), "[MM] VLM 图生文: %s", on ? "开启" : "关闭");
}

// ═══════════════════════════════════════════════════════════════════════════════
// QR 扫码生命周期 (段1出站时 active, 其余时间 inactive 零CPU)
// ═══════════════════════════════════════════════════════════════════════════════

void MissionManager::activateQrNodes()
{
  if (!qr_enable_client_ || !qr_enable_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(),
      "[MM] qr_decoder_zbar/enable 服务不可用, 跳过 QR 启用");
    return;
  }
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = true;
  qr_enable_client_->async_send_request(
    req,
    [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      if (future.get()->success) {
        qr_active_ = true;
        RCLCPP_INFO(get_logger(),
          "[MM] QR 扫码: 已启用 — %s", future.get()->message.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "[MM] QR 扫码: 启用失败 — %s",
          future.get()->message.c_str());
      }
    });
}

void MissionManager::deactivateQrNodes()
{
  if (!qr_enable_client_ || !qr_enable_client_->service_is_ready()) {
    return;
  }
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = false;
  qr_enable_client_->async_send_request(
    req,
    [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      if (future.get()->success) {
        qr_active_ = false;
        RCLCPP_INFO(get_logger(),
          "[MM] QR 扫码: 已停用, CPU 释放 — %s", future.get()->message.c_str());
      }
    });
}

// ═══════════════════════════════════════════════════════════════════════════════
// 完成回调 (transient_local, 启车前已订阅, 不丢消息)
// ═══════════════════════════════════════════════════════════════════════════════

void MissionManager::seg1_3ReadyCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  seg1_3_ready_ = msg->data;
  RCLCPP_INFO(get_logger(), "[MM] 段1预检 ready=%s", msg->data ? "true" : "false");
}

void MissionManager::seg1_3OutDoneCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "[MM] 收到 seg1_3 phase_done=%s", msg->data ? "true" : "false");
  if (msg->data) {
    seg1_3_out_done_ = true;
  }
}

void MissionManager::seg1_3DoneCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "[MM] 收到 seg1_3 done=%s", msg->data ? "true" : "false");
  if (msg->data) {
    seg1_3_done_ = true;
  }
}

void MissionManager::seg2DoneCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    RCLCPP_INFO(get_logger(), "[MM] 收到 seg2 finished=true");
    seg2_done_ = true;
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// cmd_vel
// ═══════════════════════════════════════════════════════════════════════════════

void MissionManager::publishZeroTwist()
{
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
}

}  // namespace origincar_mission_manager

// ── main ────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<origincar_mission_manager::MissionManager>());
  rclcpp::shutdown();
  return 0;
}
