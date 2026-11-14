// stage2_state_machine.hpp -- 段2 状态机: 环道循迹 + YOLO锥桶避障 (总控)
// =====================================================================
// 在段2惯导循迹(origincar_inertial_nav/path_follower)之上叠加避障能力:
//   - 按 launch 参数 direction(cw/ccw) 选顺/逆时针环路。
//   - 本节点是 /cmd_vel 的唯一发布者:
//       * 循迹态(FOLLOW): 转发 path_follower 指令, 叠加 YOLO 锥桶避障。
//   - path_follower 到点后发 /path_follower/finished=true, 本节点据此进入 FINISHED。
//
// YOLO 锥桶避障 (2026-07-22):
//   YOLO BPU 推理 → /yolo_detections → yolo_cone_bridge → /cone_detected + /cone_bearing + /cone_distance
//   锥桶在触发距内: 往内环猛转+降速, 固定时长后回循迹 (单车道内环侧最安全)

#ifndef ORIGINCAR_STAGE2__STAGE2_STATE_MACHINE_HPP_
#define ORIGINCAR_STAGE2__STAGE2_STATE_MACHINE_HPP_

#include <atomic>
#include <limits>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace origincar_stage2
{

enum class Stage2State
{
  INIT,          // 等待就绪, 开局调用 path_follower start_following 一次
  FOLLOW,        // 循迹 + YOLO 锥桶避障: 转发循迹指令, 有锥桶时往反方向绕行
  AVOID_APF,     // (已弃用) 旧 APF 避障态, FTG 连续避障后不再进入; 保留仅为兼容
  FINISHED       // 到点停车
};

const char * to_string(Stage2State s);

class Stage2StateMachine : public rclcpp::Node
{
public:
  Stage2StateMachine();

private:
  // ── 主循环 ──────────────────────────────────────────────
  void controlLoop();
  void transitionTo(Stage2State next);
  double elapsedInState() const;
  void publishTwist(double v, double omega);

  // ── 回调 ────────────────────────────────────────────────
  void onFollowCmd(const geometry_msgs::msg::Twist::SharedPtr msg);
  void onConeDetected(const std_msgs::msg::Bool::SharedPtr msg);
  void onConeDistance(const std_msgs::msg::Float32::SharedPtr msg);
  void onConeBearing(const std_msgs::msg::Float32::SharedPtr msg);
  void onFinished(const std_msgs::msg::Bool::SharedPtr msg);
  void onDirectionSignal(const std_msgs::msg::Int32::SharedPtr msg);  // QR方向信号

  // ── Mission Manager 门控: ~/enable 放行 (未放行前不发 /cmd_vel、不 start) ──
  void onEnable(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp);

  // ── path_follower 起动 ──────────────────────────────────
  void tryStartFollowing();

  // ── 通信 ────────────────────────────────────────────────
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr follow_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cone_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cone_dist_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cone_bearing_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr finished_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr direction_sub_;  // QR方向
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reload_client_;  // path_follower ~/reload_csv
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr switch_path_pub_;  // path_follower ~/switch_path
  std::shared_ptr<rclcpp::AsyncParametersClient> follower_params_client_;  // 切方向时同步改参数
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ── 参数 ────────────────────────────────────────────────
  std::string direction_;          // "cw"/"ccw"
  double turn_sign_;               // 避让侧符号: ccw=+1(先左), cw=-1(先右)
  double control_rate_;

  // ── 方向参数切换 (cw=实车调优, ccw=原始默认, 2026-07-23) ──
  void applyDirectionParams();  // 按当前 direction_ 切锥桶+循迹参数

  // ── 开环锥桶避障参数 (运行时由 applyDirectionParams 更新) ──
  bool   cone_avoid_enable_{true};     // 总开关
  double cone_avoid_max_omega_{2.5};   // 避让角速度 (rad/s), 方向由 turn_sign_ 定
  double cone_avoid_creep_v_{0.15};    // 避让前进速度 (m/s)
  double cone_avoid_duration_s_{1.4};  // 避让持续时间 (s)
  double cone_avoid_trigger_m_{0.5};   // 触发距离 (m): 锥桶<此距才触发避障

  // ── 动态切路径 (QR 奇顺偶逆) ────────────────────────────
  std::string path_cw_;            // 顺时针环道 CSV 路径
  std::string path_ccw_;           // 逆时针环道 CSV 路径
  std::string selected_csv_;       // 根据 QR 方向选出的 CSV 路径
  bool csv_switched_{false};       // 是否已通知 path_follower 切路径
  int  csv_switch_wait_{0};        // 切路径后等待 tick 数 (等 topic 送达)

  // ── 运行状态 ────────────────────────────────────────────
  Stage2State state_{Stage2State::INIT};
  rclcpp::Time state_enter_time_;
  geometry_msgs::msg::Twist last_follow_cmd_;
  bool cone_detected_{false};
  double cone_distance_{std::numeric_limits<double>::infinity()};
  double cone_bearing_{std::numeric_limits<double>::quiet_NaN()};  // rad,右负左正; NaN=丢失
  bool follow_finished_{false};
  bool start_requested_{false};
  bool start_confirmed_{false};

  // ── 开环避障状态 ──────────────────────────────────────
  bool   avoiding_{false};             // 是否正在避障
  int    avoid_ticks_{0};              // 已避障 ticks
  int    avoid_total_ticks_{0};        // 避障总 ticks (= duration * control_rate)
  int    avoid_cooldown_ticks_{0};     // 退出避障后冷却 ticks (防重复触发)

  // ── 门控 (Mission Manager 主控) ─────────────────────────
  // enabled_=false 时本节点完全静默: 不发 /cmd_vel, 不 start path_follower。
  // 由 ~/enable 服务放行 (standalone 时 auto_enable=true 自放行)。
  std::atomic<bool> enabled_{false};
  bool auto_enable_{true};
  int finish_release_ticks_{0};    // FINISHED 后补发几帧零速再彻底松手 /cmd_vel
};

}  // namespace origincar_stage2

#endif  // ORIGINCAR_STAGE2__STAGE2_STATE_MACHINE_HPP_
