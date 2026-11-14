// ============================================================================
// QR decoder (OpenCV WeChatQRCode) — 普通 Node 版 (已去除 lifecycle 依赖)
//
// 改为普通 rclcpp::Node + ~/enable (SetBool) 服务:
//   - 启动后默认启用, 由 mission_manager 按需开关
//   - enable=true  → rgbCallback 正常解码 WeChatQRCode
//   - enable=false → rgbCallback 立即返回, 零 CPU (detector 不跑)
//
// 去 Lifecycle 原因:
//   nav2_lifecycle_manager 在 Humble 中对 standalone lifecycle 节点调用 get_state
//   偶发 async_send_request 失败 (竞赛条件), 导致节点卡在 inactive 无法激活。
//   普通 Node + SetBool 服务更可靠, 不需要 lifecycle_manager_qr 中间层。
//
// 2026/07/23: ZBar → OpenCV WeChatQRCode 迁移
//   - 识别率更高 (多角度/光照适应性强)
//   - 无需额外系统库 (libzbar), 仅需含 contrib 的 OpenCV
//   - 模型文件从 model_dir 参数读取 (默认 /usr/share/wechat_qrcode)
// ============================================================================

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/wechat_qrcode.hpp"

#include <memory>
#include <string>

class QrCodeDetection : public rclcpp::Node
{
public:
  QrCodeDetection()
  : rclcpp::Node("qr_decoder_zbar")
  {
    // ── 参数声明 ──────────────────────────────────────────
    this->declare_parameter<std::string>("rgb_topic", "/aurora/rgb/image_raw");
    this->declare_parameter<std::string>("depth_topic", "/aurora/depth/image_raw");
    this->declare_parameter<bool>("enable_depth_filter", true);
    this->declare_parameter<int>("depth_min_mm", 50);
    this->declare_parameter<int>("depth_max_mm", 4000);
    this->declare_parameter<int>("min_qr_length", 1);
    this->declare_parameter<bool>("clahe_enable", true);
    this->declare_parameter<bool>("sharpen_enable", true);
    this->declare_parameter<double>("fx", 421.307);
    this->declare_parameter<double>("fy", 422.254);
    this->declare_parameter<double>("cx", 320.27);
    this->declare_parameter<double>("cy", 197.01);
    this->declare_parameter<std::string>("model_dir", "/usr/share/wechat_qrcode");

    // ── 读取参数 ──────────────────────────────────────────
    rgb_topic_ = this->get_parameter("rgb_topic").as_string();
    depth_topic_ = this->get_parameter("depth_topic").as_string();
    enable_depth_filter_ = this->get_parameter("enable_depth_filter").as_bool();
    depth_min_mm_ = this->get_parameter("depth_min_mm").as_int();
    depth_max_mm_ = this->get_parameter("depth_max_mm").as_int();
    min_qr_length_ = this->get_parameter("min_qr_length").as_int();
    fx_ = this->get_parameter("fx").as_double();
    fy_ = this->get_parameter("fy").as_double();
    cx_ = this->get_parameter("cx").as_double();
    cy_ = this->get_parameter("cy").as_double();

    std::string model_dir = this->get_parameter("model_dir").as_string();
    std::string detect_prototxt = model_dir + "/detect.prototxt";
    std::string detect_caffemodel = model_dir + "/detect.caffemodel";
    std::string sr_prototxt = model_dir + "/sr.prototxt";
    std::string sr_caffemodel = model_dir + "/sr.caffemodel";

    // ── WeChatQRCode detector 初始化 ──────────────────────
    try {
      detector_ = cv::makePtr<cv::wechat_qrcode::WeChatQRCode>(
        detect_prototxt, detect_caffemodel,
        sr_prototxt, sr_caffemodel);
      RCLCPP_INFO(this->get_logger(),
        "WeChatQRCode detector 初始化成功: model_dir=%s", model_dir.c_str());
    } catch (const cv::Exception &e) {
      RCLCPP_FATAL(this->get_logger(),
        "WeChatQRCode 初始化失败: %s (model_dir=%s). "
        "请下载模型文件: "
        "https://github.com/WeChatCV/opencv_3rdparty/tree/wechat_qrcode",
        e.what(), model_dir.c_str());
      throw;
    }

    // ── 预处理: CLAHE 提对比度 + unsharp 锐化 ──────────────
    clahe_enable_ = this->get_parameter("clahe_enable").as_bool();
    sharpen_enable_ = this->get_parameter("sharpen_enable").as_bool();
    if (clahe_enable_) { clahe_ = cv::createCLAHE(2.0, cv::Size(8, 8)); }

    // ── 创建 pub/sub (普通 Publisher, 无需激活) ──────────────
    rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      rgb_topic_, 10,
      std::bind(&QrCodeDetection::rgbCallback, this, std::placeholders::_1));

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, 10,
      std::bind(&QrCodeDetection::depthCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&QrCodeDetection::odomCallback, this, std::placeholders::_1));

    sign_com_pub = this->create_publisher<std_msgs::msg::String>("/sign", 10);
    sign_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/sign_pose", 10);

    frame_count_ = 0;
    last_qr_frame_ = -1;
    consecutive_rejects_ = 0;

    // ── ~/enable 服务 (SetBool): 由 mission_manager 按需开关 ───
    enable_srv_ = this->create_service<std_srvs::srv::SetBool>(
      "~/enable",
      std::bind(&QrCodeDetection::onEnable, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
      "qr_decoder 就绪 (WeChatQRCode): rgb=%s depth=%s filter=%s range=[%d,%d]mm "
      "min_qr_len=%d fx=%.1f fy=%.1f cx=%.1f cy=%.1f enabled=%s "
      "scan=深度自适应(近场1x→2x 远场>2m:2x→4x)+CLAHE+锐化",
      rgb_topic_.c_str(), depth_topic_.c_str(),
      enable_depth_filter_ ? "on" : "off",
      depth_min_mm_, depth_max_mm_, min_qr_length_,
      fx_, fy_, cx_, cy_,
      enabled_ ? "YES" : "NO");
  }

private:
  // ── ~/enable 服务回调 ───────────────────────────────────
  void onEnable(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> resp)
  {
    bool prev = enabled_;
    enabled_ = req->data;
    resp->success = true;
    resp->message = enabled_ ? "QR decoder enabled (WeChatQRCode active)"
                             : "QR decoder disabled (WeChatQRCode idle, CPU released)";
    if (prev != enabled_) {
      RCLCPP_INFO(this->get_logger(),
        "qr_decoder %s", enabled_ ? "ENABLED — WeChatQRCode 解码开始" : "DISABLED — WeChatQRCode 解码停止, CPU 释放");
    }
  }

  // ═══════════════════════════════════════════════════════════════
  // 回调 (与原逻辑完全一致)
  // ═══════════════════════════════════════════════════════════════

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_x_ = msg->pose.pose.position.x;
    odom_y_ = msg->pose.pose.position.y;
    double qw = msg->pose.pose.orientation.w;
    double qz = msg->pose.pose.orientation.z;
    odom_yaw_ = std::atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz);
  }

  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // 即使 disabled 也缓存深度图 (zero-cost, 只存引用)
    try {
      latest_depth_ = cv_bridge::toCvCopy(msg, "mono16")->image;
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge depth error: %s", e.what());
    }
  }

  // 估算图像中心区域深度中值 (m), 用于自适应上采样; 无有效深度返回 -1
  double estimateCenterDepth()
  {
    if (latest_depth_.empty()) { return -1.0; }
    const int w = latest_depth_.cols, h = latest_depth_.rows;
    // 中心 40% ROI
    const int rx = w * 3 / 10, ry = h * 3 / 10;
    const int rw = w * 4 / 10, rh = h * 4 / 10;
    if (rw <= 0 || rh <= 0) { return -1.0; }
    cv::Rect roi(rx, ry, rw, rh);
    cv::Mat patch = latest_depth_(roi).clone();
    // 展平, 筛有效值 (>0)
    std::vector<uint16_t> vals;
    vals.reserve(rw * rh);
    for (int y = 0; y < patch.rows; ++y) {
      const uint16_t *row = patch.ptr<uint16_t>(y);
      for (int x = 0; x < patch.cols; ++x) {
        if (row[x] > 0) { vals.push_back(row[x]); }
      }
    }
    if (vals.empty()) { return -1.0; }
    // 中值 (抵抗离群噪声)
    std::nth_element(vals.begin(), vals.begin() + vals.size() / 2, vals.end());
    return static_cast<double>(vals[vals.size() / 2]) / 1000.0;  // mm → m
  }

  // 扫描一张(可能已缩放)的灰度图。命中则把符号中心映射回原图像素坐标
  // (orig = offset + scaled/scale), 做深度校验并发布 /sign + /sign_pose。
  // 返回是否至少发布了一个符号。
  bool scanAndPublish(const cv::Mat & gray_img, double scale, int ox, int oy)
  {
    if (gray_img.empty()) { return false; }
    if (!detector_) { return false; }

    std::vector<cv::Mat> points;
    std::vector<std::string> results;
    try {
      results = detector_->detectAndDecode(gray_img, points);
    } catch (const cv::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "WeChatQRCode detectAndDecode error: %s", e.what());
      return false;
    }

    if (results.empty()) { return false; }

    bool published = false;
    for (size_t i = 0; i < results.size(); i++) {
      std::string qr_data = results[i];

      // Layer 1: 内容长度校验
      if (static_cast<int>(qr_data.length()) < min_qr_length_) {
        consecutive_rejects_++;
        RCLCPP_WARN(this->get_logger(),
          "[qr_decoder] REJECT(length): '%s' len=%zu < min=%d (rejects=%d)",
          qr_data.c_str(), qr_data.length(), min_qr_length_, consecutive_rejects_);
        continue;
      }

      // 符号中心 → 从四角点计算中心, 映射回原图像素坐标
      double sx = 0.0, sy = 0.0;
      int num_corners = 0;
      if (i < points.size() && !points[i].empty()) {
        cv::Mat corners = points[i];  // N×2 float32 (通常 4 个角点)
        for (int r = 0; r < corners.rows; r++) {
          sx += corners.at<float>(r, 0);
          sy += corners.at<float>(r, 1);
        }
        num_corners = corners.rows;
      }
      const int denom = std::max(1, num_corners);
      const int cx_px = static_cast<int>((sx / denom) / scale);
      const int cy_px = static_cast<int>((sy / denom) / scale);
      const int cx_img = ox + cx_px;
      const int cy_img = oy + cy_px;

      // Layer 2: 深度校验 (advisory)
      int depth_mm = 0;
      bool depth_valid = false;
      if (enable_depth_filter_ && !latest_depth_.empty()) {
        if (cx_img >= 0 && cx_img < latest_depth_.cols &&
            cy_img >= 0 && cy_img < latest_depth_.rows) {
          depth_mm = latest_depth_.at<uint16_t>(cy_img, cx_img);
          depth_valid = (depth_mm >= depth_min_mm_ && depth_mm <= depth_max_mm_);
        }
      }

      consecutive_rejects_ = 0;
      if (depth_valid) {
        double Z = depth_mm / 1000.0;
        double X = (cx_img - cx_) * Z / fx_;
        double Y = (cy_img - cy_) * Z / fy_;
        RCLCPP_INFO(this->get_logger(),
          "[QR] %s | 3D:(%+.3f, %+.3f, %.3f)m | px(%d,%d) D:%dmm | scale=%.2f",
          qr_data.c_str(), X, Y, Z, cx_img, cy_img, depth_mm, scale);
      } else if (enable_depth_filter_) {
        RCLCPP_INFO(this->get_logger(),
          "[QR] %s | depth=%dmm (advisory) | scale=%.2f",
          qr_data.c_str(), depth_mm, scale);
      } else {
        RCLCPP_INFO(this->get_logger(), "[QR] %s | depth filter off | scale=%.2f",
          qr_data.c_str(), scale);
      }

      auto sign_com_msg = std_msgs::msg::String();
      sign_com_msg.data = qr_data;
      sign_com_pub->publish(std::move(sign_com_msg));

      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.stamp = this->now();
      pose_msg.header.frame_id = "odom";
      pose_msg.pose.position.x = odom_x_;
      pose_msg.pose.position.y = odom_y_;
      pose_msg.pose.orientation.z = std::sin(odom_yaw_ / 2.0);
      pose_msg.pose.orientation.w = std::cos(odom_yaw_ / 2.0);
      sign_pose_pub_->publish(pose_msg);

      RCLCPP_INFO(this->get_logger(),
        "[QR] %s | robot_pos: (%.2f, %.2f, yaw=%.2f)",
        qr_data.c_str(), odom_x_, odom_y_, odom_yaw_);
      published = true;
    }
    return published;
  }

  void rgbCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // ★ 门控: disabled 时立即返回, detector 不跑, CPU 零占用
    if (!enabled_) { return; }

    cv::Mat frame;
    try {
      frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge RGB error: %s", e.what());
      return;
    }

    if (frame.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Empty RGB frame.");
      return;
    }

    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    frame_count_++;

    // ── 预处理: CLAHE 提对比度 + unsharp 锐化 (治低对比/轻模糊) ──
    cv::Mat proc = gray;
    if (clahe_enable_ && clahe_) {
      cv::Mat tmp;
      clahe_->apply(proc, tmp);
      proc = tmp;
    }
    if (sharpen_enable_) {
      cv::Mat blur;
      cv::GaussianBlur(proc, blur, cv::Size(0, 0), 1.5);
      cv::addWeighted(proc, 1.5, blur, -0.5, 0.0, proc);
    }

    // ── 深度自适应扫描: 按深度中值自动选上采样倍率 ──
    // 中心40%ROI深度中值: <1m→原图, 1-2m→2x, >2m→2x→4x
    double depth_m = estimateCenterDepth();
    bool hit = false;

    if (depth_m > 0 && depth_m >= 2.0) {
      // 远场 (>2m): 跳过原图, 直接2x→4x
      cv::Mat up2x;
      cv::resize(proc, up2x, cv::Size(), 2.0, 2.0, cv::INTER_CUBIC);
      hit = scanAndPublish(up2x, 2.0, 0, 0);
      if (!hit) {
        cv::Mat up4x;
        cv::resize(proc, up4x, cv::Size(), 4.0, 4.0, cv::INTER_CUBIC);
        hit = scanAndPublish(up4x, 4.0, 0, 0);
      }
    } else {
      // 近场/中场/无深度: 原图→2x (默认行为)
      hit = scanAndPublish(proc, 1.0, 0, 0);
      if (!hit) {
        cv::Mat up2x;
        cv::resize(proc, up2x, cv::Size(), 2.0, 2.0, cv::INTER_CUBIC);
        hit = scanAndPublish(up2x, 2.0, 0, 0);
      }
    }

    if (hit) {
      if (last_qr_frame_ < 0 || (frame_count_ - last_qr_frame_) > 5) {
        RCLCPP_INFO(this->get_logger(),
          "[qr_decoder] QR DETECTED frame=%d (gap=%d frames)",
          frame_count_, last_qr_frame_ < 0 ? -1 : frame_count_ - last_qr_frame_ - 1);
      }
      last_qr_frame_ = frame_count_;
    }

    // Periodic scanning status
    if (frame_count_ % 200 == 0) {
      int gap = last_qr_frame_ < 0 ? -1 : frame_count_ - last_qr_frame_;
      RCLCPP_INFO(this->get_logger(),
        "[qr_decoder] SCAN_STATUS frame=%d last_qr_frame=%d gap=%d frames rejects=%d",
        frame_count_, last_qr_frame_, gap, consecutive_rejects_);
    }
  }

  // ═══════════════════════════════════════════════════════════════
  // 成员变量
  // ═══════════════════════════════════════════════════════════════

  std::string rgb_topic_;
  std::string depth_topic_;
  bool enable_depth_filter_;
  int depth_min_mm_;
  int depth_max_mm_;
  int min_qr_length_;
  double fx_, fy_, cx_, cy_;
  cv::Mat latest_depth_;

  // ★ WeChatQRCode detector (替代 ZBar ImageScanner)
  cv::Ptr<cv::wechat_qrcode::WeChatQRCode> detector_;

  // ★ 改为普通 Publisher (不再需要 on_activate)
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sign_com_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr sign_pose_pub_;
  double odom_x_ = 0.0, odom_y_ = 0.0, odom_yaw_ = 0.0;
  int frame_count_ = 0;
  int last_qr_frame_ = -1;
  int consecutive_rejects_ = 0;
  bool clahe_enable_{true};
  bool sharpen_enable_{true};
  cv::Ptr<cv::CLAHE> clahe_;

  // ★ enable/disable 门控
  std::atomic<bool> enabled_{true};  // 默认启用
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;
};

// ═══════════════════════════════════════════════════════════════════
// main — 普通 spin, 无 lifecycle 依赖
// ═══════════════════════════════════════════════════════════════════
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<QrCodeDetection>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
