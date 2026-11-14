// stage2_state_machine.cpp -- 段2 状态机实现 (开环避障版)
#include "origincar_stage2/stage2_state_machine.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

using namespace std::chrono_literals;

namespace origincar_stage2
{

const char * to_string(Stage2State s)
{
  switch (s) {
    case Stage2State::INIT:      return "INIT";
    case Stage2State::FOLLOW:    return "FOLLOW";
    case Stage2State::AVOID_APF: return "AVOID_APF";
    case Stage2State::FINISHED:  return "FINISHED";
  }
  return "UNKNOWN";
}

Stage2StateMachine::Stage2StateMachine()
: rclcpp::Node("stage2_state_machine")
{
  // ── 基础参数 ──────────────────────────────────────────
  direction_    = declare_parameter<std::string>("direction", "cw");
  control_rate_ = declare_parameter<double>("control_rate", 50.0);
  turn_sign_    = (direction_ == "ccw") ? 1.0 : -1.0;

  // QR 扫描方向信号 (transient_local, 晚启动也能收到)
  rclcpp::QoS sign_qos(1);
  sign_qos.transient_local();
  direction_sub_ = create_subscription<std_msgs::msg::Int32>(
    "/sign_switch_sign_data", sign_qos,
    std::bind(&Stage2StateMachine::onDirectionSignal, this, std::placeholders::_1));

  // ── 开环锥桶避障参数 (2026-07-22) ──────────────────
  cone_avoid_enable_     = declare_parameter<bool>("cone_avoid_enable", true);
  cone_avoid_max_omega_  = declare_parameter<double>("cone_avoid_max_omega", 2.5);
  cone_avoid_creep_v_    = declare_parameter<double>("cone_avoid_creep_v", 0.15);
  cone_avoid_duration_s_ = declare_parameter<double>("cone_avoid_duration_s", 1.4);
  cone_avoid_trigger_m_  = declare_parameter<double>("cone_avoid_trigger_m", 0.5);
  avoid_total_ticks_     = static_cast<int>(cone_avoid_duration_s_ * control_rate_);

  // ── 动态切路径 (QR 奇顺偶逆) ──────────────────────────
  path_cw_   = declare_parameter<std::string>("path_cw", "");
  path_ccw_  = declare_parameter<std::string>("path_ccw", "");
  selected_csv_ = path_cw_;  // 默认 CW, 收到 QR 方向后再改

  // ── 话题名 ────────────────────────────────────────────
  const std::string cmd_vel_topic =
    declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  const std::string follow_cmd_topic =
    declare_parameter<std::string>("follow_cmd_topic", "/path_follower/cmd_vel");
  const std::string cone_topic =
    declare_parameter<std::string>("cone_topic", "/cone_detected");
  const std::string cone_dist_topic =
    declare_parameter<std::string>("cone_dist_topic", "/cone_distance");
  const std::string cone_bearing_topic =
    declare_parameter<std::string>("cone_bearing_topic", "/cone_bearing");
  const std::string finished_topic =
    declare_parameter<std::string>("finished_topic", "/path_follower/finished");
  const std::string start_service =
    declare_parameter<std::string>("start_service", "/path_follower/start_following");

  // ── 通信 ──────────────────────────────────────────────
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

  follow_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    follow_cmd_topic, 10,
    std::bind(&Stage2StateMachine::onFollowCmd, this, std::placeholders::_1));

  cone_sub_ = create_subscription<std_msgs::msg::Bool>(
    cone_topic, 10,
    std::bind(&Stage2StateMachine::onConeDetected, this, std::placeholders::_1));

  cone_dist_sub_ = create_subscription<std_msgs::msg::Float32>(
    cone_dist_topic, 10,
    std::bind(&Stage2StateMachine::onConeDistance, this, std::placeholders::_1));

  cone_bearing_sub_ = create_subscription<std_msgs::msg::Float32>(
    cone_bearing_topic, 10,
    std::bind(&Stage2StateMachine::onConeBearing, this, std::placeholders::_1));

  rclcpp::QoS finished_qos(1);
  finished_qos.transient_local();
  finished_sub_ = create_subscription<std_msgs::msg::Bool>(
    finished_topic, finished_qos,
    std::bind(&Stage2StateMachine::onFinished, this, std::placeholders::_1));

  start_client_ = create_client<std_srvs::srv::Trigger>(start_service);

  // 动态切路径: 通知 path_follower 切换 CSV (QR 奇顺偶逆)
  switch_path_pub_ = create_publisher<std_msgs::msg::String>(
    "/path_follower/switch_path", 10);
  reload_client_ = create_client<std_srvs::srv::Trigger>(
    "/path_follower/reload_csv");

  // 切方向时同步更新 path_follower 参数 (curvature_ff_gain, lookahead_k)
  follower_params_client_ =
    std::make_shared<rclcpp::AsyncParametersClient>(this, "path_follower");

  // ── 门控: Mission Manager 主控时 auto_enable=false, 由 ~/enable 放行 ──
  auto_enable_ = declare_parameter<bool>("auto_enable", true);
  enabled_.store(auto_enable_);
  enable_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/enable",
    std::bind(&Stage2StateMachine::onEnable, this,
      std::placeholders::_1, std::placeholders::_2));

  state_enter_time_ = now();

  const double rate = std::max(1.0, control_rate_);
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / rate),
    std::bind(&Stage2StateMachine::controlLoop, this));

  RCLCPP_INFO(get_logger(),
    "stage2 状态机就绪: direction=%s(内环=%s), 控制频率=%.0fHz, "
    "开环锥桶避障(触发距=%.2fm max_ω=%.1f creep=%.2f 持续%.1fs)",
    direction_.c_str(), (turn_sign_ > 0 ? "左" : "右"), rate,
    cone_avoid_trigger_m_, cone_avoid_max_omega_, cone_avoid_creep_v_, cone_avoid_duration_s_);
}

double Stage2StateMachine::elapsedInState() const
{
  return (now() - state_enter_time_).seconds();
}

void Stage2StateMachine::transitionTo(Stage2State next)
{
  if (next == state_) {
    return;
  }
  RCLCPP_INFO(get_logger(), "状态: %s -> %s", to_string(state_), to_string(next));
  state_ = next;
  state_enter_time_ = now();
}

void Stage2StateMachine::publishTwist(double v, double omega)
{
  geometry_msgs::msg::Twist t;
  t.linear.x = v;
  t.angular.z = omega;
  cmd_pub_->publish(t);
}

// ── 回调 ────────────────────────────────────────────────
void Stage2StateMachine::onFollowCmd(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  last_follow_cmd_ = *msg;
}

void Stage2StateMachine::onConeDetected(const std_msgs::msg::Bool::SharedPtr msg)
{
  cone_detected_ = msg->data;
}

void Stage2StateMachine::onConeDistance(const std_msgs::msg::Float32::SharedPtr msg)
{
  cone_distance_ = std::isfinite(msg->data)
    ? static_cast<double>(msg->data)
    : std::numeric_limits<double>::infinity();
}

void Stage2StateMachine::onConeBearing(const std_msgs::msg::Float32::SharedPtr msg)
{
  // 保留 NaN(=方位丢失), 供避让侧选择与退出判定区分"正前方"和"无方位信息"。
  // 旧实现把 NaN 压成 0.0, 会被误当成"锥桶在正前", 破坏退出/选侧逻辑。
  cone_bearing_ = static_cast<double>(msg->data);
}

void Stage2StateMachine::onFinished(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    follow_finished_ = true;
  }
}

// ── 按方向切换参数: 本节点锥桶参数 + path_follower 循迹参数 ──
void Stage2StateMachine::applyDirectionParams()
{
  // cw=实车调优值 (2026-07-23), ccw=原始默认
  const bool is_cw = (direction_ == "cw");
  const double curvature_ff_gain = is_cw ? 0.20 : 0.35;
  const double lookahead_k       = is_cw ? 0.50 : 0.80;

  // 更新本节点锥桶避障参数 (CW/CCW 统一)
  cone_avoid_creep_v_    = 0.40;
  cone_avoid_duration_s_ = 0.60;
  cone_avoid_max_omega_  = 5.00;
  avoid_total_ticks_ = static_cast<int>(cone_avoid_duration_s_ * control_rate_);

  RCLCPP_INFO(get_logger(),
    "[QR] 方向参数已切换: %s → creep_v=%.2f dur=%.1fs max_ω=%.1f "
    "curvature_ff=%.2f lookahead_k=%.2f",
    direction_.c_str(), cone_avoid_creep_v_, cone_avoid_duration_s_,
    cone_avoid_max_omega_, curvature_ff_gain, lookahead_k);

  // 同步更新 path_follower 参数 (fire-and-forget)
  if (follower_params_client_->service_is_ready()) {
    auto params = std::vector<rclcpp::Parameter>{
      rclcpp::Parameter("curvature_ff_gain", curvature_ff_gain),
      rclcpp::Parameter("lookahead_k", lookahead_k)};
    follower_params_client_->set_parameters(
      params,
      [this](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
        auto results = future.get();
        bool ok = !results.empty() && results[0].successful;
        RCLCPP_INFO(get_logger(),
          "[QR] path_follower 参数同步: %s",
          ok ? "ok" : "fail");
      });
  }
}

// ── QR 方向信号 (来自 qr_decision, /sign_switch_sign_data: 3=CW, 4=CCW) ─
void Stage2StateMachine::onDirectionSignal(const std_msgs::msg::Int32::SharedPtr msg)
{
  // QR 奇顺偶逆: data==3=CW(奇), data==4=CCW(偶)
  const std::string new_dir = (msg->data == 4) ? "ccw" : "cw";
  if (new_dir != direction_) {
    direction_ = new_dir;
    turn_sign_ = (direction_ == "ccw") ? 1.0 : -1.0;
    selected_csv_ = (direction_ == "ccw") ? path_ccw_ : path_cw_;
    RCLCPP_INFO(get_logger(),
      "[QR] 收到方向信号: %d → %s (避让侧=%s, CSV=%s)",
      msg->data, direction_.c_str(),
      turn_sign_ > 0 ? "左" : "右",
      selected_csv_.c_str());

    // ★ 通知 path_follower 切路径
    if (!selected_csv_.empty()) {
      auto switch_msg = std_msgs::msg::String();
      switch_msg.data = selected_csv_;
      switch_path_pub_->publish(switch_msg);
      csv_switched_ = true;
      csv_switch_wait_ = 5;
      RCLCPP_INFO(get_logger(),
        "[QR] 已通知 path_follower 切路径: %s (等待 %d ticks)",
        selected_csv_.c_str(), csv_switch_wait_);
    }

    // ★ 切方向相关参数 (锥桶避障 + 循迹曲率)
    applyDirectionParams();
  }
}

// ── Mission Manager 门控: ~/enable 放行 ─────────────────
void Stage2StateMachine::onEnable(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
{
  // 复位状态并放行 (幂等: 重复 enable 也安全)
  state_ = Stage2State::INIT;
  state_enter_time_ = now();
  start_requested_ = false;
  start_confirmed_ = false;
  follow_finished_ = false;
  finish_release_ticks_ = 0;
  csv_switch_wait_ = 0;
  avoiding_ = false;
  avoid_ticks_ = 0;
  avoid_cooldown_ticks_ = 0;
  enabled_.store(true);
  resp->success = true;
  resp->message = "stage2 enabled: INIT";
  RCLCPP_INFO(get_logger(), "[MM] 收到 enable → 放行段2 (INIT→循迹)");
}

// ── path_follower 起动 ──────────────────────────────────
void Stage2StateMachine::tryStartFollowing()
{
  if (start_requested_) {
    return;
  }
  if (!start_client_->service_is_ready()) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "等待 path_follower start 服务上线...");
    return;
  }
  start_requested_ = true;
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  start_client_->async_send_request(
    req,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      auto resp = future.get();
      if (resp->success) {
        start_confirmed_ = true;
        RCLCPP_INFO(get_logger(), "path_follower 已发车: %s", resp->message.c_str());
      } else {
        start_requested_ = false;
        RCLCPP_WARN(get_logger(), "start_following 失败, 重试: %s",
          resp->message.c_str());
      }
    });
}

// ── 主循环: 状态机 ──────────────────────────────────────
void Stage2StateMachine::controlLoop()
{
  // 门控: 未放行前完全静默, 绝不发布 /cmd_vel, 不触发 path_follower。
  // 段1(Nav2)/段3(Nav2) 期间由此让出 /cmd_vel, 避免和 Nav2 打架。
  if (!enabled_.load()) {
    return;
  }

  switch (state_) {
    case Stage2State::INIT: {
      publishTwist(0.0, 0.0);

      // ★ 切路径等待: csv_switched_ 后需等几帧让 topic 送达 path_follower,
      //   确保 start_following → _begin() 时 csv_path 已更新。
      if (csv_switched_ && csv_switch_wait_ > 0) {
        --csv_switch_wait_;
        if (csv_switch_wait_ == 0) {
          RCLCPP_INFO(get_logger(), "切路径等待完成: %s, 准备发车", selected_csv_.c_str());
        }
        break;  // 还在等, 本帧不 tryStart
      }

      tryStartFollowing();
      if (start_confirmed_) {
        transitionTo(Stage2State::FOLLOW);
      }
      break;
    }

    case Stage2State::FOLLOW: {
      if (follow_finished_) {
        transitionTo(Stage2State::FINISHED);
        break;
      }
      double v = last_follow_cmd_.linear.x;
      double omega = last_follow_cmd_.angular.z;

      // ── 开环锥桶避障: YOLO检测到锥桶 → 立刻往内环猛转+降速, 固定时长后回循迹 ──
      if (cone_avoid_enable_) {
        // 冷却计数 (退出避障后短暂屏蔽, 防重复触发)
        if (avoid_cooldown_ticks_ > 0) {
          --avoid_cooldown_ticks_;
        }

        if (!avoiding_ && avoid_cooldown_ticks_ == 0 && cone_detected_ &&
            std::isfinite(cone_distance_) && cone_distance_ < cone_avoid_trigger_m_) {
          // ★ 锥桶进入触发距 → 往内环侧猛转 (单车道, 内环最安全)
          avoiding_ = true;
          avoid_ticks_ = 0;
          RCLCPP_WARN(get_logger(),
            "[YOLO] 锥桶距=%.2fm < %.2fm → 开环避障: 往内环%s转 ω=%.1f v=%.2f 持续%.1fs",
            cone_distance_, cone_avoid_trigger_m_,
            (turn_sign_ > 0 ? "左" : "右"),
            cone_avoid_max_omega_, cone_avoid_creep_v_, cone_avoid_duration_s_);
        }

        if (avoiding_) {
          ++avoid_ticks_;
          if (avoid_ticks_ >= avoid_total_ticks_) {
            // 避障结束, 回循迹
            avoiding_ = false;
            avoid_cooldown_ticks_ = static_cast<int>(0.5 * control_rate_);  // 0.5s冷却
            RCLCPP_INFO(get_logger(),
              "[YOLO] 避障结束 (%.1fs) → 回循迹, 冷却%.1fs",
              cone_avoid_duration_s_, 0.5);
          } else {
            // 执行避障: 往内环猛转 + 降速
            v = cone_avoid_creep_v_;
            omega = turn_sign_ * cone_avoid_max_omega_;
            // cw: turn_sign_=-1 → omega负(右转); ccw: turn_sign_=+1 → omega正(左转)

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
              "[YOLO] 避障中: tick=%d/%d ω=%.1f v=%.2f [往内环%s]",
              avoid_ticks_, avoid_total_ticks_, omega, v,
              (turn_sign_ > 0 ? "左" : "右"));
          }
        }
      }

      publishTwist(v, omega);
      break;
    }

    case Stage2State::AVOID_APF: {
      // 已弃用: Gap-Follow 连续避障并入 FOLLOW, 不再进入本态。
      // 防御性处理: 万一被外部置入, 直接回到 FOLLOW。
      transitionTo(Stage2State::FOLLOW);
      break;
    }

    case Stage2State::FINISHED: {
      // 补发几帧零速把车稳停, 然后彻底松手 /cmd_vel, 交回上层 (段3 Nav2)
      if (finish_release_ticks_ < 10) {
        publishTwist(0.0, 0.0);
        ++finish_release_ticks_;
      } else {
        enabled_.store(false);
      }
      break;
    }
  }
}

}  // namespace origincar_stage2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<origincar_stage2::Stage2StateMachine>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
