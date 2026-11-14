// ============================================================================
// origincar_compition —— 竞赛任务状态机
//
// 流程: 起点初始化位姿 → 去任务发布点(途中监控 /qr_event + 起点→任务点连线进度)
//       → (扫到码且过半 立即改道 / 或 到达任务点) → 硬编码倒车右转把车头复位到
//       起点朝向 → Nav2 前往黄色通道入口(到达朝向=起点) → 完成。
// 复位机动: 阿克曼车无法原地转; 与其让 Nav2 在窄空间做脆弱的 180° 调头, 不如先取消
//           Nav2 目标, 直接向 /cmd_vel 发 linear.x<0 + angular.z>0 (右打舵倒车,
//           车尾右后摆), 闭环读 TF 偏航角, 车头回到起点朝向即停。此后再交回 Nav2
//           去入口路点——此时那段退化为一次平顺前进弧线, 到达时朝向仍= 起点朝向。
// ============================================================================

#include "origincar_compition/mission_state_machine.hpp"

#include <cmath>
#include <vector>

#include "rclcpp/parameter_client.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace origincar_compition
{

using std::placeholders::_1;
using std::placeholders::_2;
using NavigateToPose = MissionStateMachine::NavigateToPose;

MissionStateMachine::MissionStateMachine()
: rclcpp::Node("mission_state_machine")
{
  // ---- 参数 ----
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
  qr_event_topic_ = declare_parameter<std::string>("qr_event_topic", "/qr_event");
  half_progress_threshold_ = declare_parameter<double>("half_progress_threshold", 0.5);
  amcl_settle_sec_ = declare_parameter<double>("amcl_settle_sec", 2.0);
  amcl_settle_inbound_sec_ = declare_parameter<double>("amcl_settle_inbound_sec", 3.0);
  inbound_reinit_pose_ = declare_parameter<bool>("inbound_reinit_pose", false);
  enable_startup_escape_ = declare_parameter<bool>("enable_startup_escape", true);
  startup_escape_speed_ = declare_parameter<double>("startup_escape_speed", 0.15);
  startup_escape_distance_ = declare_parameter<double>("startup_escape_distance", 0.18);
  startup_escape_timeout_sec_ = declare_parameter<double>("startup_escape_timeout_sec", 3.5);
  startup_escape_settle_sec_ = declare_parameter<double>("startup_escape_settle_sec", 0.4);
  reverse_initial_heading_ = declare_parameter<bool>("reverse_initial_heading", true);

  // 复位机动 (倒车右转) 参数 —— 均可在 mission_params.yaml 调
  reorient_reverse_speed_ = declare_parameter<double>("reorient_reverse_speed", -0.15);
  reorient_turn_rate_ = declare_parameter<double>("reorient_turn_rate", 0.6);
  reorient_yaw_tolerance_ = declare_parameter<double>("reorient_yaw_tolerance", 0.10);
  reorient_settle_sec_ = declare_parameter<double>("reorient_settle_sec", 1.2);
  reorient_timeout_sec_ = declare_parameter<double>("reorient_timeout_sec", 10.0);
  reorient_max_reverse_dist_ =
    declare_parameter<double>("reorient_max_reverse_dist", 0.5);
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  skip_seg2_ = declare_parameter<bool>("skip_seg2", false);
  hard_brake_sec_ = declare_parameter<double>("hard_brake_sec", 0.3);

  auto load_wp = [this](const std::string & key, const std::vector<double> & def) {
    auto v = declare_parameter<std::vector<double>>(key, def);
    Waypoint wp;
    if (v.size() >= 3) { wp.x = v[0]; wp.y = v[1]; wp.yaw = v[2]; }
    else { RCLCPP_ERROR(get_logger(), "参数 '%s' 需要 [x,y,yaw] 三元素", key.c_str()); }
    return wp;
  };
  start_ = load_wp("start", {-1.985, -0.014, 0.262});
  task_ = load_wp("task", {0.48, -3.16, 0.0});
  yellow_ = load_wp("yellow", {0.843, -1.078, 0.262});   // +x 5cm 微偏 (见 mission_params.yaml)
  // P 点 = 起点系内 x=-0.20m, y=+0.20m + 反向偏航角 (入站终点, 停偏补偿; 07/22 迭代)
  parking_ = load_wp("parking", {-1.844, 0.231,
    []() -> double {
      double y = 0.262 + M_PI;
      while (y >  M_PI) { y -= 2.0 * M_PI; }
      while (y < -M_PI) { y += 2.0 * M_PI; }
      return y;
    }()});  // 默认: 起点 yaw + π ≈ -2.880

  // 入口到点朝向精确对准: 动态设 controller_server 的 goal_checker.yaw_goal_tolerance
  controller_server_name_ =
    declare_parameter<std::string>("controller_server_name", "/controller_server");
  entry_yaw_tolerance_ = declare_parameter<double>("entry_yaw_tolerance", 0.05);
  loose_yaw_tolerance_ = declare_parameter<double>("loose_yaw_tolerance", 3.14);
  entry_xy_tolerance_ = declare_parameter<double>("entry_xy_tolerance", 0.10);
  loose_xy_tolerance_ = declare_parameter<double>("loose_xy_tolerance", 0.25);
  controller_params_client_ =
    std::make_shared<rclcpp::AsyncParametersClient>(this, controller_server_name_);

  // ---- 接口 ----
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
  initialpose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", rclcpp::QoS(10));
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  qr_sub_ = create_subscription<std_msgs::msg::String>(
    qr_event_topic_, 10, std::bind(&MissionStateMachine::qrCallback, this, _1));

  // 过半场时发屏显 (qr_decision 不再发, 延迟到 reorient 触发点)
  qr_result_pub_ = create_publisher<std_msgs::msg::String>("/qr_result", 10);

  // Mission Manager 接口
  start_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/start", std::bind(&MissionStateMachine::onStartService, this, _1, _2));
  start_inbound_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/start_inbound", std::bind(&MissionStateMachine::onStartInboundService, this, _1, _2));
  rclcpp::QoS done_qos(1);
  done_qos.transient_local();
  phase_done_pub_ = create_publisher<std_msgs::msg::Bool>("~/phase_done", done_qos);
  done_pub_ = create_publisher<std_msgs::msg::Bool>("~/done", done_qos);
  ready_pub_ = create_publisher<std_msgs::msg::Bool>("~/ready", done_qos);
  publishReady(false);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 启动序列 (2Hz 轮询: 等 server → 发 initialpose → 等 AMCL → 发任务目标)
  startup_timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&MissionStateMachine::startupOnce, this));
  // 进度监控 (10Hz)
  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&MissionStateMachine::controlLoop, this));

  RCLCPP_INFO(get_logger(),
    "mission_state_machine 启动: start(%.2f,%.2f,yaw%.3f) task(%.2f,%.2f) qr=%s thr=%.2f "
    "复位机动: 倒车右转 v=%.2f ω=%.2f 容差=%.2frad settle=%.1fs 超时=%.1fs → %s",
    start_.x, start_.y, start_.yaw, task_.x, task_.y,
    qr_event_topic_.c_str(), half_progress_threshold_,
    reorient_reverse_speed_, reorient_turn_rate_, reorient_yaw_tolerance_,
    reorient_settle_sec_, reorient_timeout_sec_, cmd_vel_topic_.c_str());
  RCLCPP_INFO(get_logger(),
    "段1起点脱困=%s: v=%.2fm/s 距离=%.2fm 超时=%.1fs 停车settle=%.1fs",
    enable_startup_escape_ ? "开启" : "关闭", startup_escape_speed_,
    startup_escape_distance_, startup_escape_timeout_sec_, startup_escape_settle_sec_);
}

// ---------------------------------------------------------------------------
// Mission Manager 接口: ~/start 重置并启动, ~/done 通知完成
// ---------------------------------------------------------------------------
void MissionStateMachine::onStartService(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
{
  if (!preflight_ready_) {
    resp->success = false;
    resp->message = "preflight not complete; wait for ~/ready=true";
    RCLCPP_WARN(get_logger(), "[MM] 拒绝 start：预检尚未完成");
    return;
  }

  // 预检已完成 → 进入 departure (startup_phase_=2)
  preflight_ready_ = false;
  publishReady(false);
  state_ = MissionState::INIT;
  startup_phase_ = 2;
  startup_escape_start_valid_ = false;
  qr_detected_ = false;
  reorient_started_ = false;
  task_goal_handle_.reset();

  // 重新创建启动定时器 (旧的已 cancel)
  startup_timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&MissionStateMachine::startupOnce, this));
  // 重新创建进度监控定时器
  if (control_timer_) { control_timer_->cancel(); }
  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&MissionStateMachine::controlLoop, this));

  resp->success = true;
  resp->message = "departure sequence started";
  RCLCPP_INFO(get_logger(), "[MM] ⚡ 收到 start 指令 → 发车!");
}

void MissionStateMachine::publishDone()
{
  std_msgs::msg::Bool msg;
  msg.data = (state_ == MissionState::DONE);
  done_pub_->publish(msg);
  RCLCPP_INFO(get_logger(), "[MM] 发送 done=%s", msg.data ? "true" : "false");
}

// ---------------------------------------------------------------------------
// 启动序列
// ---------------------------------------------------------------------------
void MissionStateMachine::startupOnce()
{
  const MissionState current = state_.load();
  if (current != MissionState::INIT && current != MissionState::STARTUP_ESCAPE) { return; }

  if (!nav_client_->action_server_is_ready()) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
      "等待 navigate_to_pose action server...");
    return;
  }

  if (startup_phase_ == 0) {
    publishInitialPose(start_);
    settle_deadline_ = now() + rclcpp::Duration::from_seconds(amcl_settle_sec_);
    startup_phase_ = 1;
    RCLCPP_INFO(get_logger(), "已发布 /initialpose(起点), 等待 AMCL 收敛 %.1fs...", amcl_settle_sec_);
    return;
  }

  if (startup_phase_ == 1) {
    if (now() < settle_deadline_) { return; }

    // AMCL 收敛完成 → 发布 ready, 取消定时器, 等待 /mission_manager/start
    preflight_ready_ = true;
    publishReady(true);
    startup_timer_->cancel();
    RCLCPP_INFO(get_logger(),
      "段1预检全部完成，车辆保持零速；等待发车指令 /mission_manager/start");
    return;
  }

  // startup_phase_ == 2: beginDeparture (由 onStartService 触发)
  if (startup_phase_ == 2) {
    // 若关闭脱困, 直接发任务目标
    if (!enable_startup_escape_) {
      RCLCPP_INFO(get_logger(), "出发 → 任务发布点 (%.2f, %.2f)", task_.x, task_.y);
      sendGoal(task_, /*force_start_yaw=*/false, MissionState::GO_TO_TASK_POINT);
      startup_timer_->cancel();
      return;
    }

    // ★ STARTUP_ESCAPE: 低速直行驶离 P 点致命代价区
    state_ = MissionState::STARTUP_ESCAPE;
    startup_phase_ = 3;
    startup_escape_start_valid_ = lookupRobotXY(
      startup_escape_start_x_, startup_escape_start_y_);
    startup_escape_deadline_ = now() +
      rclcpp::Duration::from_seconds(startup_escape_timeout_sec_);
    RCLCPP_WARN(get_logger(),
      "段1发车脱困: 沿起点朝向低速直行 %.2fm (v=%.2fm/s, 超时%.1fs)",
      startup_escape_distance_, startup_escape_speed_, startup_escape_timeout_sec_);
    return;
  }

  // startup_phase_ == 3: 直行脱困
  if (startup_phase_ == 3) {
    if (!startup_escape_start_valid_) {
      startup_escape_start_valid_ = lookupRobotXY(
        startup_escape_start_x_, startup_escape_start_y_);
    }

    double moved = 0.0;
    bool distance_reached = false;
    if (startup_escape_start_valid_) {
      double cx, cy;
      if (lookupRobotXY(cx, cy)) {
        moved = std::hypot(
          cx - startup_escape_start_x_, cy - startup_escape_start_y_);
        distance_reached = moved >= startup_escape_distance_;
      }
    }

    const bool timed_out = now() >= startup_escape_deadline_;
    if (distance_reached || timed_out) {
      for (int i = 0; i < 5; ++i) { publishZeroTwist(); }
      startup_phase_ = 4;
      settle_deadline_ = now() +
        rclcpp::Duration::from_seconds(startup_escape_settle_sec_);
      RCLCPP_INFO(get_logger(),
        "段1脱困停止: %s, TF位移=%.2fm; %.1fs后交还 Nav2 去任务点",
        distance_reached ? "已驶离起点致命栅格" : "超时安全停车",
        moved, startup_escape_settle_sec_);
      return;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = startup_escape_speed_;
    cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd);
    return;
  }

  // startup_phase_ == 4: 停车 settle, 然后交还 Nav2
  if (startup_phase_ == 4) {
    publishZeroTwist();
    if (now() < settle_deadline_) { return; }
    RCLCPP_INFO(get_logger(), "出发 → 任务发布点 (%.2f, %.2f)", task_.x, task_.y);
    sendGoal(task_, /*force_start_yaw=*/false, MissionState::GO_TO_TASK_POINT);
    startup_timer_->cancel();
  }
}

void MissionStateMachine::publishInitialPose(const Waypoint & wp)
{
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.frame_id = map_frame_;
  msg.header.stamp = now();
  msg.pose.pose.position.x = wp.x;
  msg.pose.pose.position.y = wp.y;
  msg.pose.pose.orientation = quatFromYaw(wp.yaw);
  // 合理的初始协方差 (x,y ~0.25m², yaw ~0.068rad²), 与 RViz 2D Pose Estimate 一致
  msg.pose.covariance[0] = 0.25;
  msg.pose.covariance[7] = 0.25;
  msg.pose.covariance[35] = 0.06853891945200942;
  initialpose_pub_->publish(msg);
}

// ---------------------------------------------------------------------------
// 发送导航目标 (任务发布点 / 复位后的黄色通道入口)
//   force_start_yaw=true: 到达朝向强制 = 起点朝向 (黄色通道入口, 供视觉巡线对准)
// ---------------------------------------------------------------------------
// 动态设 controller_server 的 goal_checker.yaw_goal_tolerance。异步不阻塞:
// goal_checker 每次判定都读该参数, 只要在车到点前落地即可(距离远, ms级延迟无碍)。
void MissionStateMachine::setYawGoalTolerance(double tol)
{
  if (!controller_params_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(),
      "%s 参数服务未就绪, 跳过设 yaw_goal_tolerance=%.3f (到点朝向按当前容差)",
      controller_server_name_.c_str(), tol);
    return;
  }
  controller_params_client_->set_parameters(
    {rclcpp::Parameter("goal_checker.yaw_goal_tolerance", tol)},
    [this, tol](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> f) {
      const auto & res = f.get();
      if (!res.empty() && res.front().successful) {
        RCLCPP_INFO(get_logger(), "已设 goal_checker.yaw_goal_tolerance=%.3f rad", tol);
      } else {
        RCLCPP_WARN(get_logger(),
          "设 goal_checker.yaw_goal_tolerance=%.3f 失败(参数名/命名空间?)", tol);
      }
    });
}

void MissionStateMachine::setXYGoalTolerance(double tol)
{
  if (!controller_params_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(),
      "%s 参数服务未就绪, 跳过设 xy_goal_tolerance=%.3f (到点位置按当前容差)",
      controller_server_name_.c_str(), tol);
    return;
  }
  controller_params_client_->set_parameters(
    {rclcpp::Parameter("goal_checker.xy_goal_tolerance", tol)},
    [this, tol](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> f) {
      const auto & res = f.get();
      if (!res.empty() && res.front().successful) {
        RCLCPP_INFO(get_logger(), "已设 goal_checker.xy_goal_tolerance=%.3f m", tol);
      } else {
        RCLCPP_WARN(get_logger(),
          "设 goal_checker.xy_goal_tolerance=%.3f 失败(参数名/命名空间?)", tol);
      }
    });
}

void MissionStateMachine::sendGoal(const Waypoint & wp, bool force_start_yaw,
                                   MissionState next_state)
{
  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = map_frame_;
  goal.pose.header.stamp = now();
  goal.pose.pose.position.x = wp.x;
  goal.pose.pose.position.y = wp.y;
  goal.pose.pose.orientation = quatFromYaw(force_start_yaw ? start_.yaw : wp.yaw);

  const bool is_task = (next_state == MissionState::GO_TO_TASK_POINT);

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
  opts.goal_response_callback =
    [this](GoalHandle::SharedPtr gh) {
      if (!gh) {
        RCLCPP_ERROR(get_logger(), "导航目标被拒绝");
        state_ = MissionState::FAILED;
        publishDone();
        return;
      }
      task_goal_handle_ = gh;   // 当前活动 Nav2 目标句柄 (任务点或黄色入口)
    };
  opts.result_callback =
    is_task
      ? std::function<void(const GoalHandle::WrappedResult &)>(
          std::bind(&MissionStateMachine::taskGoalResult, this, _1))
      : std::function<void(const GoalHandle::WrappedResult &)>(
          std::bind(&MissionStateMachine::yellowGoalResult, this, _1));

  state_ = next_state;
  nav_client_->async_send_goal(goal, opts);
}

// ---------------------------------------------------------------------------
// QR 回调: 收到任意已校验事件即置位
// ---------------------------------------------------------------------------
void MissionStateMachine::qrCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data.empty()) { return; }
  if (!qr_detected_.exchange(true)) {
    qr_data_ = msg->data;
    RCLCPP_INFO(get_logger(), "[QR] 收到二维码事件: '%s' → 即刻屏显", msg->data.c_str());

    // 立刻屏显 (不等半场)
    auto display = std_msgs::msg::String();
    display.data = qr_data_;
    qr_result_pub_->publish(display);
  }
}

// ---------------------------------------------------------------------------
// 控制循环 (10Hz): 仅在去任务点阶段计算进度并判定改道
// ---------------------------------------------------------------------------
void MissionStateMachine::controlLoop()
{
  if (state_.load() != MissionState::GO_TO_TASK_POINT || reorient_started_) { return; }

  // 扫码即刻触发, 不等半场
  if (qr_detected_) {
    RCLCPP_INFO(get_logger(),
      "触发改道: 已扫码 → 取消任务点, 开始倒车右转复位");
    startReorient();
  }
}

// ---------------------------------------------------------------------------
// 扫码后复位机动: 硬编码倒车右转, 闭环把车头复位到起点朝向
// ---------------------------------------------------------------------------
void MissionStateMachine::startReorient()
{
  if (reorient_started_.exchange(true)) { return; }   // 幂等: 两个触发点最多进一次
  state_ = MissionState::REORIENT;

  if (control_timer_) { control_timer_->cancel(); }
  if (startup_timer_) { startup_timer_->cancel(); }

  // 取消可能仍在执行的 Nav2 任务目标 (fire-and-forget; 自然到点时句柄已完成/为空)
  if (task_goal_handle_) {
    nav_client_->async_cancel_goal(task_goal_handle_);
  }

  const rclcpp::Time t0 = now();
  settle_deadline_ = t0 + rclcpp::Duration::from_seconds(reorient_settle_sec_);
  // 超时只计驱动窗口 (settle 之后再算 timeout 秒)
  timeout_deadline_ = settle_deadline_ + rclcpp::Duration::from_seconds(reorient_timeout_sec_);

  // 记录倒车起点 (map系), 用于限制倒车位移, 防止一路倒进诊疗室禁区
  reorient_start_valid_ = lookupRobotXY(reorient_start_x_, reorient_start_y_);

  // 记录初始偏航误差, 便于现场核对倒车右转是否是可达方向
  double yaw0;
  if (lookupRobotYaw(yaw0)) {
    double d0 = start_.yaw - yaw0;
    d0 = std::atan2(std::sin(d0), std::cos(d0));
    RCLCPP_INFO(get_logger(),
      "复位开始: 取消 Nav2, settle %.1fs; 当前 yaw=%.3f 目标=%.3f 误差=%.3frad",
      reorient_settle_sec_, yaw0, start_.yaw, d0);
  } else {
    RCLCPP_INFO(get_logger(), "复位开始: 取消 Nav2, settle %.1fs (TF 暂不可用)",
      reorient_settle_sec_);
  }

  // 20Hz 机动定时器, 与 now() 同钟 (兼容 sim time)
  reorient_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(0.05),
    std::bind(&MissionStateMachine::reorientLoop, this));
}

void MissionStateMachine::reorientLoop()
{
  const rclcpp::Time t = now();

  // 阶段1: settle —— 不发任何指令, 让 velocity_smoother 超时归零并停发, 独占 /cmd_vel
  if (t < settle_deadline_) { return; }

  // 阶段2: 倒车右转 + 闭环按 yaw 判停
  double yaw;
  if (!lookupRobotYaw(yaw)) {
    publishZeroTwist();                              // TF 缺失时保持安全
    if (t > timeout_deadline_) { finishReorient(false); }
    return;
  }

  double d = start_.yaw - yaw;
  d = std::atan2(std::sin(d), std::cos(d));           // 最短带符号角差
  if (std::abs(d) <= reorient_yaw_tolerance_) { finishReorient(true);  return; }
  if (t > timeout_deadline_)                  { finishReorient(false); return; }

  // 安全边界: 倒车位移超过上限即停 (防止朝向未达时一路倒进诊疗室禁区)
  if (reorient_start_valid_) {
    double cx, cy;
    if (lookupRobotXY(cx, cy)) {
      const double moved = std::hypot(cx - reorient_start_x_, cy - reorient_start_y_);
      if (moved > reorient_max_reverse_dist_) {
        RCLCPP_WARN(get_logger(),
          "复位倒车位移 %.2fm 超上限 %.2fm → 提前停止, 交回 Nav2 (防倒入禁区)",
          moved, reorient_max_reverse_dist_);
        finishReorient(false);
        return;
      }
    }
  }

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = reorient_reverse_speed_;             // 负 = 倒车
  cmd.angular.z = reorient_turn_rate_;                // 正 = 车头逆时针 (右打舵倒车)
  cmd_vel_pub_->publish(cmd);
}

void MissionStateMachine::finishReorient(bool reached)
{
  for (int i = 0; i < 5; ++i) { publishZeroTwist(); } // 连发多条, 单条可能丢
  if (reorient_timer_) { reorient_timer_->cancel(); }

  if (reached) {
    RCLCPP_INFO(get_logger(), "复位完成: 车头已对准起点朝向 → Nav2 前往黄色通道入口 (%.2f, %.2f)",
      yellow_.x, yellow_.y);
  } else {
    RCLCPP_WARN(get_logger(),
      "复位超时: 朝向可能未完全对齐, 仍尝试 Nav2 前往黄色通道入口 (%.2f, %.2f)",
      yellow_.x, yellow_.y);
  }
  // 入口到点精确对准(修联合交接): 非保底模式收紧 yaw+xy 容差。
  // 保底模式(skip_seg2)无需精确对准——段2不跑, 到达入口后立刻归程。
  if (!skip_seg2_) {
    setYawGoalTolerance(entry_yaw_tolerance_);
    setXYGoalTolerance(entry_xy_tolerance_);
  } else {
    // 放回宽松容差, Nav2 判定到点即停, 不纠结对齐
    setYawGoalTolerance(loose_yaw_tolerance_);
    setXYGoalTolerance(loose_xy_tolerance_);
  }
  // 朝向已(大致)复位, 去入口路点这一段退化为一次平顺的前进弧线, Nav2 到达时朝向=起点
  sendGoal(yellow_, /*force_start_yaw=*/true, MissionState::GO_TO_YELLOW_CHANNEL);
}

void MissionStateMachine::publishZeroTwist()
{
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
}

// ---------------------------------------------------------------------------
// 结果回调
// ---------------------------------------------------------------------------
void MissionStateMachine::taskGoalResult(const GoalHandle::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      // 未提前触发就到达了任务点 → 同样进入倒车右转复位
      RCLCPP_INFO(get_logger(), "已到达任务发布点 (未提前触发) → 开始倒车右转复位");
      startReorient();
      break;
    case rclcpp_action::ResultCode::CANCELED:
      // 多为 startReorient() 自身取消所致, 忽略 (复位流程已接管)
      break;
    case rclcpp_action::ResultCode::ABORTED:
    default:
      if (!reorient_started_) {
        RCLCPP_ERROR(get_logger(), "去任务发布点失败 (ABORTED)");
        state_ = MissionState::FAILED;
        publishDone();
      }
      break;
  }
}

void MissionStateMachine::yellowGoalResult(const GoalHandle::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      state_ = MissionState::OUTBOUND_DONE;
      RCLCPP_INFO(get_logger(), "✅ 出站完成: 已到达黄色通道入口 → 等待入站指令 (~/start_inbound)");
      publishPhaseDone();
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(get_logger(), "去黄色通道入口失败 (ABORTED)");
      state_ = MissionState::FAILED;
      publishDone();
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(get_logger(), "去黄色通道入口被取消");
      break;
    default:
      break;
  }
}

// ---------------------------------------------------------------------------
// 工具
// ---------------------------------------------------------------------------
geometry_msgs::msg::Quaternion MissionStateMachine::quatFromYaw(double yaw) const
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion out;
  out.x = q.x(); out.y = q.y(); out.z = q.z(); out.w = q.w();
  return out;
}

bool MissionStateMachine::lookupRobotXY(double & x, double & y)
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
      "TF %s→%s 不可用: %s", map_frame_.c_str(), base_frame_.c_str(), ex.what());
    return false;
  }
  x = tf.transform.translation.x;
  y = tf.transform.translation.y;
  return true;
}

bool MissionStateMachine::lookupRobotYaw(double & yaw)
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "复位: TF %s→%s 不可用: %s", map_frame_.c_str(), base_frame_.c_str(), ex.what());
    return false;
  }
  const auto & q = tf.transform.rotation;
  const double siny = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  yaw = std::atan2(siny, cosy);
  return true;
}

double MissionStateMachine::projectionFraction(const Waypoint & s, const Waypoint & t,
                                               double px, double py)
{
  const double dx = t.x - s.x;
  const double dy = t.y - s.y;
  const double len2 = dx * dx + dy * dy;
  if (len2 < 1e-9) { return 1.0; }  // 起点=任务点, 视为已到
  return ((px - s.x) * dx + (py - s.y) * dy) / len2;
}

// ---------------------------------------------------------------------------
// 出站完成: 通知 Mission Manager (入站 (归程) 的入口已到达)
// ---------------------------------------------------------------------------
void MissionStateMachine::publishPhaseDone()
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  phase_done_pub_->publish(msg);
  RCLCPP_INFO(get_logger(), "[MM] 发送 phase_done=true (出站完成, 等待入站指令)");
}

void MissionStateMachine::publishReady(bool ready)
{
  std_msgs::msg::Bool msg;
  msg.data = ready;
  ready_pub_->publish(msg);
}

// ---------------------------------------------------------------------------
// Mission Manager 接口: ~/start_inbound 触发归程
// ---------------------------------------------------------------------------
void MissionStateMachine::onStartInboundService(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
{
  if (state_ != MissionState::OUTBOUND_DONE) {
    resp->success = false;
    resp->message = "not in OUTBOUND_DONE state";
    RCLCPP_WARN(get_logger(), "[MM] start_inbound 被拒绝: 当前状态=%s", toString(state_.load()));
    return;
  }

  // 启动入站序列 (2Hz 轮询: 等 server → 发 initialpose → 等 AMCL → 发 P 点目标)
  inbound_timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&MissionStateMachine::inboundStartupOnce, this));
  // 重置启动分步
  startup_phase_ = 0;

  resp->success = true;
  resp->message = "inbound sequence started";
  RCLCPP_INFO(get_logger(), "[MM] 收到 start_inbound → 启动归程序列");
}

// ---------------------------------------------------------------------------
// 入站启动序列 (从黄色入口 → P 点)
// ---------------------------------------------------------------------------
void MissionStateMachine::inboundStartupOnce()
{
  if (state_ != MissionState::OUTBOUND_DONE) { return; }

  if (!nav_client_->action_server_is_ready()) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
      "入站: 等待 navigate_to_pose action server...");
    return;
  }

  if (startup_phase_ == 0) {
    // ★ 默认路径: AMCL 段2全程 active 已跟踪到当前位姿, 跳过重发 /initialpose 与
    //   amcl_settle_inbound_sec 重收敛, 直接发 P 点目标 —— 消除段2→段3 的停顿。
    if (!inbound_reinit_pose_) {
      RCLCPP_INFO(get_logger(),
        "入站: 跳过重发 initialpose (AMCL 全程 active) → 直接出发 P 点 (%.2f, %.2f, yaw=%.3f)",
        parking_.x, parking_.y, parking_.yaw);
      sendStartGoal(parking_);
      inbound_timer_->cancel();
      return;
    }

    // 回退路径 (inbound_reinit_pose=true): 重发 initialpose(入口, 背对起点) + 等 AMCL
    Waypoint init_pose = yellow_;
    if (reverse_initial_heading_) {
      init_pose.yaw = start_.yaw + M_PI;
      while (init_pose.yaw >  M_PI) { init_pose.yaw -= 2.0 * M_PI; }
      while (init_pose.yaw < -M_PI) { init_pose.yaw += 2.0 * M_PI; }
    }
    publishInitialPose(init_pose);
    settle_deadline_ = now() + rclcpp::Duration::from_seconds(amcl_settle_inbound_sec_);
    startup_phase_ = 1;
    RCLCPP_INFO(get_logger(),
      "入站: 已发 /initialpose(黄色入口, yaw=%.3f%s), 等待 AMCL 收敛 %.1fs...",
      init_pose.yaw, reverse_initial_heading_ ? ", 背对起点" : "", amcl_settle_inbound_sec_);
    return;
  }

  if (startup_phase_ == 1) {
    if (now() < settle_deadline_) { return; }
    RCLCPP_INFO(get_logger(), "入站: 出发 → P 点 (%.2f, %.2f, yaw=%.3f)", parking_.x, parking_.y, parking_.yaw);
    sendStartGoal(parking_);
    inbound_timer_->cancel();
  }
}

// ---------------------------------------------------------------------------
// 发送归程导航目标 (P 点)。到达朝向=parking_.yaw (与起点相反), 配合 loose_yaw_tolerance 仍按宽松判定。
// ---------------------------------------------------------------------------
void MissionStateMachine::sendStartGoal(const Waypoint & wp)
{
  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = map_frame_;
  goal.pose.header.stamp = now();
  goal.pose.pose.position.x = wp.x;
  goal.pose.pose.position.y = wp.y;
  goal.pose.pose.orientation = quatFromYaw(wp.yaw);

  // P 点归程: 放回宽 yaw+xy 容差, 抵消入口时的双收紧, 避免 P 点因
  // use_rotate_to_heading=false 硬凑朝向/位置而到点抖/超时。
  setYawGoalTolerance(loose_yaw_tolerance_);
  setXYGoalTolerance(loose_xy_tolerance_);

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
  opts.goal_response_callback =
    [this](GoalHandle::SharedPtr gh) {
      if (!gh) {
        RCLCPP_ERROR(get_logger(), "入站: 导航目标被拒绝");
        state_ = MissionState::FAILED;
        publishDone();
        return;
      }
      task_goal_handle_ = gh;
    };
  opts.result_callback =
    std::bind(&MissionStateMachine::startGoalResult, this, _1);

  state_ = MissionState::GO_TO_START;
  nav_client_->async_send_goal(goal, opts);
}

// ---------------------------------------------------------------------------
// 归程结果回调
// ---------------------------------------------------------------------------
void MissionStateMachine::startGoalResult(const GoalHandle::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      state_ = MissionState::DONE;
      RCLCPP_INFO(get_logger(), "✅ 全部完成: 已回到 P 点 (yaw=%.3f)", parking_.yaw);
      // 立停制动: 连发零速, 防惯性滑行撞墙
      {
        auto brake_end = now() + rclcpp::Duration::from_seconds(hard_brake_sec_);
        while (now() < brake_end) {
          publishZeroTwist();
          rclcpp::sleep_for(std::chrono::milliseconds(20));
        }
        RCLCPP_INFO(get_logger(), "立停制动完成 (%.1fs)", hard_brake_sec_);
      }
      publishDone();
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(get_logger(), "归程失败 (ABORTED)");
      state_ = MissionState::FAILED;
      publishDone();
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(get_logger(), "归程被取消");
      break;
    default:
      break;
  }
}

}  // namespace origincar_compition

// ---------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<origincar_compition::MissionStateMachine>());
  rclcpp::shutdown();
  return 0;
}
