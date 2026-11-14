#ifndef ORIGINCAR_COMPITION__MISSION_STATE_MACHINE_HPP_
#define ORIGINCAR_COMPITION__MISSION_STATE_MACHINE_HPP_

#include <atomic>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "origincar_compition/mission_types.hpp"

namespace origincar_compition
{

/// 竞赛任务状态机节点。
///
/// 编排流程: 起点初始化位姿 → 去任务发布点(途中监控 /qr_event + 连线进度) →
/// (扫到码且过半 或 到达任务点) → 硬编码倒车右转把车头复位到起点朝向 →
/// Nav2 去黄色通道入口(到达朝向=起点) → 发布 ~/phase_done 等待入站指令 →
/// (收到 ~/start_inbound) → 发 initialpose(入口,背对起点) → 等 AMCL →
/// Nav2 回 P 点 → 完成。
///
/// 合并了原 seg3_state_machine 的归程逻辑，统一管理全部 Nav2 NavigateToPose。
class MissionStateMachine : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  MissionStateMachine();

private:
  // ---- 启动 / 流程 ----
  void startupOnce();                       // 一次性: 等 server → 发 initialpose → 起点脱困 → 发任务目标
  void inboundStartupOnce();                // 入站启动: 等 server → 发 initialpose(入口) → 等AMCL → 发P点目标
  void publishInitialPose(const Waypoint & wp);
  void sendGoal(const Waypoint & wp, bool force_start_yaw, MissionState next_state);
  void sendStartGoal(const Waypoint & wp);  // 发送归程导航目标 (P 点)
  // 动态设 controller_server 的 goal_checker.yaw_goal_tolerance:
  //   入口目标前收紧(精确对准朝向), P点归程目标前放回(不限朝向)。异步, 不阻塞。
  void setYawGoalTolerance(double tol);
  // 动态设 controller_server 的 goal_checker.xy_goal_tolerance (同上, 异步):
  void setXYGoalTolerance(double tol);
  void controlLoop();                       // 10Hz: 计算进度, 判定改道

  // ---- 扫码后复位机动 (硬编码倒车右转, 闭环按 yaw) ----
  void startReorient();                     // 取消 Nav2 目标 → 进入 REORIENT
  void reorientLoop();                      // 20Hz: settle → 倒车右转 → 到位/超时停车
  void finishReorient(bool reached);        // 停车 → 发黄色通道入口 Nav2 目标
  void publishZeroTwist();                  // 发一条零速 Twist

  // ---- Mission Manager 接口: ~/start 出站, ~/start_inbound 入站, 通知 ----
  void onStartService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp);
  void onStartInboundService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp);
  void publishPhaseDone();                  // OUTBOUND_DONE 时通知 MM 出站完成
  void publishReady(bool ready);            // 预检完成/发车时通知 MM
  void publishDone();                       // DONE/FAILED 时通过 transient_local 发布

  // ---- 回调 ----
  void qrCallback(const std_msgs::msg::String::SharedPtr msg);
  void taskGoalResult(const GoalHandle::WrappedResult & result);
  void yellowGoalResult(const GoalHandle::WrappedResult & result);
  void startGoalResult(const GoalHandle::WrappedResult & result);

  // ---- 工具 ----
  geometry_msgs::msg::Quaternion quatFromYaw(double yaw) const;
  bool lookupRobotXY(double & x, double & y);              // TF map→base (平面位置)
  bool lookupRobotYaw(double & yaw);                       // TF map→base (偏航角)
  static double projectionFraction(const Waypoint & s, const Waypoint & t,
                                   double px, double py);  // 连线投影分数

  // ---- 参数 ----
  std::string map_frame_;
  std::string base_frame_;
  std::string qr_event_topic_;
  double half_progress_threshold_{0.5};
  double amcl_settle_sec_{2.0};
  bool enable_startup_escape_{true};         // 仅段一发车前绕过起点致命栅格
  double startup_escape_speed_{0.15};        // m/s，沿起始朝向低速直行
  double startup_escape_distance_{0.18};     // m，达到后立即交还 Nav2
  double startup_escape_timeout_sec_{3.5};   // s，TF失效时的硬停止上限
  double startup_escape_settle_sec_{0.4};    // s，停车后再请求全局规划
  double amcl_settle_inbound_sec_{2.0};      // 入站时 AMCL 收敛等待 (可独立调)
  bool reverse_initial_heading_{true};       // 入站 initialpose 是否反向 (背对起点)
  // 入站是否重发 initialpose 并等 AMCL 重收敛。默认 false: AMCL 段2全程 active 已知位姿,
  // 跳过重发+3s重收敛可消除段2→段3的停顿。若段2环道后 AMCL 飘, 置 true 恢复旧行为。
  bool inbound_reinit_pose_{false};
  Waypoint start_, task_, yellow_, parking_;

  // 入口到点朝向精确对准: 动态设 controller_server 的 goal_checker.yaw_goal_tolerance
  std::string controller_server_name_{"/controller_server"};
  double entry_yaw_tolerance_{0.05};         // rad, 入口目标到点朝向容差(收紧)
  double loose_yaw_tolerance_{3.14};         // rad, 非入口目标放回(P点不限朝向)
  double entry_xy_tolerance_{0.10};          // m, 入口目标到点位置容差(收紧)
  double loose_xy_tolerance_{0.18};          // m, 非入口目标放回(P点宽松; 0.25→0.13→0.18)

  // ---- 复位机动参数 (倒车右转) ----
  double reorient_reverse_speed_{-0.15};   // m/s, 负=倒车
  double reorient_turn_rate_{0.6};         // rad/s, 正=车头逆时针 (倒车右打舵)
  double reorient_yaw_tolerance_{0.10};    // rad, 车头到达起点朝向的容差
  double reorient_settle_sec_{1.2};        // 取消 Nav2 后等 velocity_smoother 静默
  double reorient_timeout_sec_{10.0};      // 驱动窗口安全上限
  double reorient_max_reverse_dist_{0.5};  // 倒车复位最大位移(m), 超出即停, 防倒进禁区
  bool   skip_seg2_{false};               // 保底模式: 跳过段2, 黄色入口不收紧容差
  double hard_brake_sec_{0.3};             // 归程到P点立停制动时长(s)
  std::string cmd_vel_topic_{"/cmd_vel"};

  // ---- 状态 ----
  std::atomic<MissionState> state_{MissionState::INIT};
  std::atomic<bool> qr_detected_{false};
  std::string qr_data_;                               // QR码原始数据, 过半场时发屏显
  std::atomic<bool> reorient_started_{false};   // startReorient 幂等守卫 (取代旧 redirecting_)
  GoalHandle::SharedPtr task_goal_handle_;

  // 启动分步: 0=等Action, 1=等AMCL, 2=起点脱困中, 3=停车稳定后发任务目标
  int startup_phase_{0};
  bool preflight_ready_{false};              // AMCL收敛后置true, 等start指令
  rclcpp::Time startup_escape_deadline_;
  double startup_escape_start_x_{0.0};
  double startup_escape_start_y_{0.0};
  bool startup_escape_start_valid_{false};
  rclcpp::Time settle_deadline_;         // 启动 AMCL 收敛 / 复位 settle 共用
  rclcpp::Time timeout_deadline_;        // 复位驱动窗口截止
  double reorient_start_x_{0.0};         // 复位倒车起点 (map系), 用于限制倒车位移
  double reorient_start_y_{0.0};
  bool   reorient_start_valid_{false};   // 是否已记录倒车起点

  // ---- 接口 ----
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr qr_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr qr_result_pub_;  // 过半场时发屏显
  rclcpp::TimerBase::SharedPtr startup_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr reorient_timer_;
  rclcpp::TimerBase::SharedPtr inbound_timer_;   // 入站启动序列 (2Hz)

  // Mission Manager 接口
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_inbound_srv_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr phase_done_pub_;  // 出站完成通知
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;        // 预检完成通知

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // controller_server 参数客户端 (动态设 yaw_goal_tolerance)
  std::shared_ptr<rclcpp::AsyncParametersClient> controller_params_client_;
};

}  // namespace origincar_compition

#endif  // ORIGINCAR_COMPITION__MISSION_STATE_MACHINE_HPP_
