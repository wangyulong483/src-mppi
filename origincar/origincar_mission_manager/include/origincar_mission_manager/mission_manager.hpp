// ============================================================================
// origincar_mission_manager — 顶层任务调度者
//
// 职责: 协调段1+3 (Nav2多路点导航+归程, 由 mission_state_machine 统一执行)
//       与段2 (惯导环道循迹 + 锥桶避障) 的顺序执行, 并保证 cmd_vel 互斥。
//
// 流程 (合并后):
//   INIT → WAITING_START (预检完成、零速等待 /mission_manager/start)
//        → SEG1_3_OUT (调 mission_state_machine ~/start, 出走 P→QR→entrance)
//        → TRANSITION (零速制动)
//        → SEG2 (放行 stage2, 惯导环道)
//        → SEG1_3_IN (调 mission_state_machine ~/start_inbound, 归程 entrance→P)
//        → FINISHED
//
// 发车方式:
//   1) ros2 launch origincar_mission_manager mission.launch.py   # 预热
//   2) ros2 service call /mission_manager/start std_srvs/srv/Trigger "{}"  # 发车
//
// 子状态机接口:
//   段1+3 (mission_state_machine): ~/start (Trigger srv) ← 出站启动
//                                   ~/ready (Bool, transient_local) → 预检完成
//                                   ~/phase_done (Bool, transient_local) → 出站完成
//                                   ~/start_inbound (Trigger srv) ← 入站启动
//                                   ~/done (Bool, transient_local) → 到达P点
//   段2 (stage2_state_machine):   /stage2_state_machine/enable (Trigger srv) ← 放行
//                                  /path_follower/finished (Bool, transient_local) → 完成
//
// cmd_vel 互斥 (时序错开):
//   - SEG1_3_OUT: Nav2 发 cmd_vel (mission_state_machine 发复位指令)
//   - WAITING_START / TRANSITION / FINISHED: 本节点发零速 cmd_vel
//   - SEG2: stage2_state_machine 发 cmd_vel
//   - SEG1_3_IN: Nav2 发 cmd_vel (mission_state_machine 的归程)
// ============================================================================

#ifndef ORIGINCAR_MISSION_MANAGER__MISSION_MANAGER_HPP_
#define ORIGINCAR_MISSION_MANAGER__MISSION_MANAGER_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace origincar_mission_manager
{

/// 任务调度状态
enum class MissionPhase
{
  INIT,               // 等待所有接口就绪
  WAITING_START,      // 所有预检完成，保持零速并等待外部发车指令
  SEG1_3_OUT,         // 段1+3 出站: Nav2 P→QR→黄色入口 (mission_state_machine ~/start)
  TRANSITION,         // 过渡: 零速制动
  SEG2,               // 段2: 惯导环道循迹 + 锥桶避障
  SEG1_3_IN,          // 段1+3 入站: Nav2 黄色入口→P (mission_state_machine ~/start_inbound)
  FINISHED            // 全部完成
};

const char * to_string(MissionPhase p);

class MissionManager : public rclcpp::Node
{
public:
  MissionManager();

private:
  // ── 主循环 (10Hz) ────────────────────────────────────────
  void controlLoop();

  // ── 各阶段实现 ────────────────────────────────────────────
  void phaseInit();
  void phaseWaitingStart();
  void phaseSeg1_3Out();
  void phaseTransition();
  void phaseSeg2();
  void phaseSeg1_3In();
  void phaseFinished();

  // ── 过渡切换 ──────────────────────────────────────────────
  void transitionTo(MissionPhase next);
  double elapsedInPhase() const;

  // ── 接口 ──────────────────────────────────────────────────
  void seg1_3SendStart();        // 调 mission_state_machine ~/start (出站)
  void seg1_3SendInbound();      // 调 mission_state_machine ~/start_inbound (入站)
  void seg2SendStart();          // 调 stage2_state_machine/enable 放行
  void setConeDetector(bool on); // 段2才开锥桶检测, 其余段关闭省CPU
  void setVlmEnabled(bool on);   // 段2才开VLM推理, 其余段关闭省API费用
  void seg1_3OutDoneCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void seg1_3DoneCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void seg2DoneCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void seg1_3ReadyCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void onStartCommand(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp);
  void publishReady(bool ready);
  void publishZeroTwist();

  // ── QR 扫码生命周期 (段1出站才开, 其余段关省CPU) ──────────
  void activateQrNodes();
  void deactivateQrNodes();

  // ── 参数 ──────────────────────────────────────────────────
  bool   skip_seg2_{false};           // 保底模式: 跳过段2环道, 只跑段1+3
  double transition_duration_{0.5};
  double amcl_settle_sec_{2.0};
  double control_rate_{10.0};

  // 段2 开始后延迟多久再继续 (s): 让 path_follower 先用 map→odom 做绝对锚定。
  double seg2_amcl_pause_delay_{1.5};
  std::string cone_enable_service_{"/yolo_cone_bridge/enable"};
  std::string vlm_enable_service_{"/vlm_node/enable"};
  std::string qr_enable_service_{"/qr_decoder_zbar/enable"};

  // ── 状态 ──────────────────────────────────────────────────
  MissionPhase phase_{MissionPhase::INIT};
  rclcpp::Time phase_enter_time_;
  bool seg1_3_out_done_{false};
  bool seg1_3_done_{false};
  bool seg2_done_{false};
  bool seg2_pending_{false};
  bool seg1_3_ready_{false};
  bool qr_activation_requested_{false};
  bool qr_active_{false};
  bool start_command_received_{false};
  int final_stop_cycles_remaining_{0};

  bool seg2_amcl_pause_done_{false};   // 段2内锚定窗口是否结束

  // ── 通信接口 ──────────────────────────────────────────────
  // 段1+3 接口 (mission_state_machine)
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr seg1_3_start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr seg1_3_inbound_client_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr seg1_3_phase_done_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr seg1_3_done_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr seg1_3_ready_sub_;

  // 段2 接口 (stage2 / path_follower / cone_detector / vlm)
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr seg2_start_client_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr seg2_done_sub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr cone_enable_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr vlm_enable_client_;

  // cmd_vel (仅在 WAITING_START / TRANSITION 阶段发布零速)
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_command_srv_;

  // QR 扫码使能客户端
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr qr_enable_client_;

  // 主循环定时器
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace origincar_mission_manager

#endif  // ORIGINCAR_MISSION_MANAGER__MISSION_MANAGER_HPP_
