# origincar_qr

OriginCar 二维码扫描与决策包。由原 `qr_decoder`（C++）与 `origincar_qr`（Python）两个
辅助包合并而成，视觉流来自 **Aurora930 深度相机**。混合 `ament_cmake` + `ament_cmake_python`
构建：同一个包内既编译 C++ 解码节点，又安装 Python 决策节点。

后续将由统一状态机编排这两个节点，本包不含 launch 文件。

## 节点

### `qr_decoder`（C++ / OpenCV WeChatQRCode）—— 扫码/解码
- **订阅**
  - `/aurora/rgb/image_raw` (`sensor_msgs/Image`, bgr8) — 深度相机 RGB
  - `/aurora/depth/image_raw` (`sensor_msgs/Image`, mono16) — 深度图（3D 辅助校验）
  - `/odom` (`nav_msgs/Odometry`) — 记录解码时机器人位姿
- **发布**
  - `/sign` (`std_msgs/String`) — 解码出的二维码文本
  - `/sign_pose` (`geometry_msgs/PoseStamped`, frame `odom`) — 解码时刻位姿
- **主要参数**：`rgb_topic`、`depth_topic`、`enable_depth_filter`、`depth_min_mm`(50)、
  `depth_max_mm`(4000)、`min_qr_length`(1)、相机内参 `fx/fy/cx/cy`、
  `model_dir`(模型文件目录)
- 多尺度扫描（原图 + 0.5x）、深度过滤为 advisory（深度相机常滞后 RGB，不阻止发布）。

### `qr_decision`（Python）—— 方向决策
- **订阅**：`/sign`（来自 `qr_decoder`）、`/odom`
- **发布**
  - `/qr_event` (`std_msgs/String`) — 结构化事件 `num,方向,x,y`，供状态机订阅
  - `/qr_result` (`std_msgs/String`) — 同上，人类可读结果
  - `/sign_switch_sign_data` (`std_msgs/Int32`) — 顺时针=3 / 逆时针=4
- **逻辑**：仅接受 1~4 位纯数字（1~9999）；奇数=顺时针(CW)、偶数=逆时针(CCW)；
  相同码 `cooldown_sec`(默认 3.0s) 内去重。**不重复解码**，只做纯逻辑。

## 依赖

### 系统依赖

需系统已安装 ROS2 Humble、OpenCV（含 **opencv_contrib** 模块，提供 WeChatQRCode）、
`cv_bridge`，以及深度相机驱动 `deptrum-ros-driver-aurora930`（真机/系统层，不在本工作空间）。

> **不再依赖 libzbar**（已迁移至 OpenCV WeChatQRCode，2026/07/23）。

### WeChatQRCode 模型文件

WeChatQRCode 需要 4 个模型文件，放在 `model_dir` 参数指定的目录中（默认 `/usr/share/wechat_qrcode`）：

```bash
sudo mkdir -p /usr/share/wechat_qrcode
cd /usr/share/wechat_qrcode
sudo wget https://github.com/WeChatCV/opencv_3rdparty/raw/wechat_qrcode/detect.prototxt
sudo wget https://github.com/WeChatCV/opencv_3rdparty/raw/wechat_qrcode/detect.caffemodel
sudo wget https://github.com/WeChatCV/opencv_3rdparty/raw/wechat_qrcode/sr.prototxt
sudo wget https://github.com/WeChatCV/opencv_3rdparty/raw/wechat_qrcode/sr.caffemodel
```

模型总大小约 1MB。若目录不同，通过 `model_dir` 参数指定：
```bash
ros2 run origincar_qr qr_decoder --ros-args -p model_dir:=/path/to/models
```

## 构建

```bash
cd ~/dev_ws
colcon build --packages-select origincar_qr
source install/setup.bash
```

## 运行（相机已启动的前提下）

```bash
ros2 run origincar_qr qr_decoder    # 扫码/解码
ros2 run origincar_qr qr_decision   # 方向决策
```
