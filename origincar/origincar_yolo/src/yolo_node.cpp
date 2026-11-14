// Copyright (c) 2022，Horizon Robotics.
// origincar_yolo — 适配 origincar Aurora930 RGB 相机的 YOLOv5s BPU 锥桶检测
// 基于 racing_obstacle_detection_yolo, 改为标准 ROS Image 输入 (非共享内存)
#include <cv_bridge/cv_bridge.h>
#include <fstream>

#include "ai_msgs/msg/perception_targets.hpp"
#include "dnn_node/dnn_node.h"
#include "dnn_node/util/image_proc.h"
#include "hobot_cv/hobotcv_imgproc.h"
#include "sensor_msgs/msg/image.hpp"

#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "rapidjson/writer.h"

#include "origincar_yolo/parser.h"
#include "origincar_yolo/image_utils.h"

// 使用hobotcv resize nv12格式图片，固定图片宽高比
int ResizeNV12Img(const char* in_img_data,
                  const int& in_img_height,
                  const int& in_img_width,
                  const int& scaled_img_height,
                  const int& scaled_img_width,
                  cv::Mat& out_img,
                  float& ratio) {
  cv::Mat src(
      in_img_height * 3 / 2, in_img_width, CV_8UC1, (void*)(in_img_data));
  float ratio_w =
      static_cast<float>(in_img_width) / static_cast<float>(scaled_img_width);
  float ratio_h =
      static_cast<float>(in_img_height) / static_cast<float>(scaled_img_height);
  float dst_ratio = std::max(ratio_w, ratio_h);
  int resized_width, resized_height;
  if (dst_ratio == ratio_w) {
    resized_width = scaled_img_width;
    resized_height = static_cast<float>(in_img_height) / dst_ratio;
  } else if (dst_ratio == ratio_h) {
    resized_width = static_cast<float>(in_img_width) / dst_ratio;
    resized_height = scaled_img_height;
  }

  int remain = resized_width % 16;
  if (remain != 0) {
    resized_width -= remain;
    dst_ratio = static_cast<float>(in_img_width) / resized_width;
    resized_height = static_cast<float>(in_img_height) / dst_ratio;
  }
  resized_height =
      resized_height % 2 == 0 ? resized_height : resized_height - 1;
  ratio = dst_ratio;

  return hobot_cv::hobotcv_resize(
      src, in_img_height, in_img_width, out_img, resized_height, resized_width);
}

int InitClassNames(const std::string &cls_name_file,
                   hobot::dnn_node::origincar_yolo::PTQYolo5Config &yolo5_config) {
  std::ifstream fi(cls_name_file);
  if (fi) {
    yolo5_config.class_names.clear();
    std::string line;
    while (std::getline(fi, line)) {
      yolo5_config.class_names.push_back(line);
    }
    int size = yolo5_config.class_names.size();
    if(size != yolo5_config.class_num){
      RCLCPP_ERROR(rclcpp::get_logger("YoloDetection"),
                 "class_names length %d != class_num %d", size, yolo5_config.class_num);
      return -1;
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("YoloDetection"),
                 "Cannot open cls name file: %s", cls_name_file.c_str());
    return -1;
  }
  return 0;
}

int InitClassNum(const int &class_num,
                 hobot::dnn_node::origincar_yolo::PTQYolo5Config &yolo5_config) {
  if(class_num > 0){
    yolo5_config.class_num = class_num;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("YoloDetection"), "class_num=%d invalid", class_num);
    return -1;
  }
  return 0;
}

void LoadConfig(const std::string &config_file,
                hobot::dnn_node::origincar_yolo::PTQYolo5Config &yolo5_config) {
  if (config_file.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("LoadConfig"), "Config file empty!");
    return;
  }
  std::ifstream ifs(config_file.c_str());
  if (!ifs) {
    RCLCPP_ERROR(rclcpp::get_logger("LoadConfig"), "Read config [%s] fail!", config_file.data());
    return;
  }
  rapidjson::IStreamWrapper isw(ifs);
  rapidjson::Document document;
  document.ParseStream(isw);
  if (document.HasParseError()) {
    RCLCPP_ERROR(rclcpp::get_logger("LoadConfig"), "Parse config %s failed", config_file.data());
    return;
  }
  if (document.HasMember("class_num")){
    int class_num = document["class_num"].GetInt();
    if (InitClassNum(class_num, yolo5_config) < 0) return;
  }
  if (document.HasMember("cls_names_list")) {
    std::string cls_name_file = document["cls_names_list"].GetString();
    if (InitClassNames(cls_name_file, yolo5_config) < 0) return;
  }
}

struct YoloNodeOutput : public hobot::dnn_node::DnnNodeOutput {
  float ratio = 1.0;
};

// YOLO 推理节点: 订阅 Aurora930 RGB, BPU 推理, 发布 PerceptionTargets
class YoloNode : public hobot::dnn_node::DnnNode {
 public:
  YoloNode(const std::string& node_name = "origincar_yolo",
           const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 protected:
  int SetNodePara() override;
  int PostProcess(const std::shared_ptr<hobot::dnn_node::DnnNodeOutput>&
                      node_output) override;

 private:
  int model_input_width_ = -1;
  int model_input_height_ = -1;

  rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr
      img_subscription_ = nullptr;
  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr msg_publisher_ =
      nullptr;

  std::string sub_img_topic_ = "/aurora/rgb/image_raw";
  std::string config_file_ = "config/yolov5workconfig.json";

  hobot::dnn_node::origincar_yolo::PTQYolo5Config yolo5_config_ = {
    {8, 16, 32},
    {{{10, 13}, {16, 30}, {33, 23}},
     {{30, 61}, {62, 45}, {59, 119}},
     {{116, 90}, {156, 198}, {373, 326}}},
    1,
    {"obstacle"}};

  void FeedImg(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
};

YoloNode::YoloNode(const std::string& node_name,
                   const rclcpp::NodeOptions& options)
    : hobot::dnn_node::DnnNode(node_name, options) {

  this->declare_parameter<std::string>("sub_img_topic", sub_img_topic_);
  this->declare_parameter<std::string>("config_file", config_file_);
  this->get_parameter<std::string>("sub_img_topic", sub_img_topic_);
  this->get_parameter<std::string>("config_file", config_file_);

  if (Init() != 0 ||
      GetModelInputSize(0, model_input_width_, model_input_height_) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("YoloNode"), "Init fail!");
    rclcpp::shutdown();
  }
  LoadConfig(config_file_, yolo5_config_);

  // 订阅 Aurora930 RGB (标准 sensor_msgs/Image)
  img_subscription_ =
      this->create_subscription<sensor_msgs::msg::Image>(
          sub_img_topic_, 10,
          std::bind(&YoloNode::FeedImg, this, std::placeholders::_1));

  // 发布 YOLO 检测结果
  msg_publisher_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>(
      "/yolo_detections", 10);

  RCLCPP_INFO(rclcpp::get_logger("YoloNode"),
      "origincar_yolo 就绪: sub=%s model=%dx%d config=%s",
      sub_img_topic_.c_str(), model_input_width_, model_input_height_,
      config_file_.c_str());
}

int YoloNode::SetNodePara() {
  if (!dnn_node_para_ptr_) return -1;
  std::ifstream ifs(config_file_.c_str());
  if (!ifs) {
    RCLCPP_ERROR(rclcpp::get_logger("SetNodePara"),
                 "Read config [%s] fail!", config_file_.data());
    return -1;
  }
  rapidjson::IStreamWrapper isw(ifs);
  rapidjson::Document document;
  document.ParseStream(isw);
  if (document.HasParseError()) {
    RCLCPP_ERROR(rclcpp::get_logger("SetNodePara"), "Parse config fail");
    return -1;
  }
  std::string model_file;
  if (document.HasMember("model_file")) {
    model_file = document["model_file"].GetString();
  }
  dnn_node_para_ptr_->model_file = model_file;
  dnn_node_para_ptr_->model_task_type =
      hobot::dnn_node::ModelTaskType::ModelInferType;
  dnn_node_para_ptr_->task_num = 1;
  dnn_node_para_ptr_->bpu_core_ids.push_back(HB_BPU_CORE_0);
  return 0;
}

void YoloNode::FeedImg(
    const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
  if (!rclcpp::ok() || !img_msg) return;

  auto dnn_output = std::make_shared<YoloNodeOutput>();
  dnn_output->msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->msg_header->set__frame_id(img_msg->header.frame_id);
  dnn_output->msg_header->set__stamp(img_msg->header.stamp);

  // 转换 ROS Image → BGR → NV12 Pyramid
  auto cv_img = cv_bridge::cvtColorForDisplay(
      cv_bridge::toCvShare(img_msg), "bgr8");

  std::shared_ptr<hobot::dnn_node::NV12PyramidInput> pyramid = nullptr;
  pyramid = ImageUtils::GetNV12Pyramid(
        cv_img->image, model_input_height_, model_input_width_);

  if (!pyramid) {
    RCLCPP_ERROR(rclcpp::get_logger("YoloNode"), "Get pym fail");
    return;
  }

  auto inputs =
      std::vector<std::shared_ptr<hobot::dnn_node::DNNInput>>{pyramid};

  if (Run(inputs, dnn_output, nullptr, false) < 0) {
    RCLCPP_INFO(rclcpp::get_logger("YoloNode"), "Run predict fail!");
  }
}

int YoloNode::PostProcess(
    const std::shared_ptr<hobot::dnn_node::DnnNodeOutput>& node_output) {
  if (!rclcpp::ok()) return 0;

  auto tp_start = std::chrono::system_clock::now();

  ai_msgs::msg::PerceptionTargets::UniquePtr pub_data(
      new ai_msgs::msg::PerceptionTargets());
  pub_data->set__header(*node_output->msg_header);

  std::vector<std::shared_ptr<hobot::dnn_node::origincar_yolo::YoloV5Result>>
      results;

  if (hobot::dnn_node::origincar_yolo::Parse(
          node_output, results, yolo5_config_) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("YoloNode"), "Parse fail!");
    return -1;
  }

  for (auto& rect : results) {
    if (!rect) continue;
    if (rect->xmin < 0) rect->xmin = 0;
    if (rect->ymin < 0) rect->ymin = 0;
    if (rect->xmax >= model_input_width_) {
      rect->xmax = model_input_width_ - 1;
    }
    if (rect->ymax >= model_input_height_) {
      rect->ymax = model_input_height_ - 1;
    }

    std::stringstream ss;
    ss << "det: " << rect->xmin << " " << rect->ymin << " "
       << rect->xmax << " " << rect->ymax
       << " cls=" << rect->class_name << " score=" << rect->score;
    RCLCPP_INFO(rclcpp::get_logger("YoloNode"), "%s", ss.str().c_str());

    ai_msgs::msg::Roi roi;
    roi.rect.set__x_offset(rect->xmin);
    roi.rect.set__y_offset(rect->ymin);
    roi.rect.set__width(rect->xmax - rect->xmin);
    roi.rect.set__height(rect->ymax - rect->ymin);
    roi.set__confidence(rect->score);

    ai_msgs::msg::Target target;
    target.set__type(rect->class_name);
    target.rois.emplace_back(roi);
    pub_data->targets.emplace_back(std::move(target));
  }

  // 坐标映射 (resize ratio)
  auto yolo_output =
      std::dynamic_pointer_cast<YoloNodeOutput>(node_output);
  if (yolo_output && yolo_output->ratio != 1.0) {
    for (auto& target : pub_data->targets) {
      for (auto& roi : target.rois) {
        roi.rect.x_offset *= yolo_output->ratio;
        roi.rect.y_offset *= yolo_output->ratio;
        roi.rect.width *= yolo_output->ratio;
        roi.rect.height *= yolo_output->ratio;
      }
    }
  }

  if (node_output->rt_stat) {
    pub_data->set__fps(round(node_output->rt_stat->output_fps));
    if (node_output->rt_stat->fps_updated) {
      auto tp_now = std::chrono::system_clock::now();
      auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                          tp_now - tp_start).count();
      RCLCPP_WARN(rclcpp::get_logger("YoloNode"),
                  "fps in:%.1f out:%.1f infer:%dms post:%dms",
                  node_output->rt_stat->input_fps,
                  node_output->rt_stat->output_fps,
                  node_output->rt_stat->infer_time_ms, interval);
    }
  }

  msg_publisher_->publish(std::move(pub_data));
  return 0;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YoloNode>());
  rclcpp::shutdown();
  return 0;
}
