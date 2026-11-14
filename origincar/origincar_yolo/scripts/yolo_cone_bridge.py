#!/usr/bin/env python3
"""
yolo_cone_bridge.py — YOLO PerceptionTargets → 锥桶检测接口桥接
=============================================================
订阅 origincar_yolo 发布的 /yolo_detections (ai_msgs/PerceptionTargets),
结合 Aurora930 深度图计算锥桶方位角和距离,
发布与 lidar_cone_detector 完全兼容的接口:
  /cone_detected  (Bool)     True=前方有锥桶
  /cone_distance  (Float32)  最近锥桶距离(m)
  /cone_bearing   (Float32)  最近锥桶方位角(rad), 右负左正

图像坐标系 → 方位角 (用 Aurora930 RGB 内参):
  bearing = atan2(bx - cx, fx)
  其中 bx = 检测框中心 x, cx = 光心 x, fx = 焦距 x

距离: 取检测框中心区域深度图的中值

参数:
  camera_fx / camera_fy / camera_cx / camera_cy  (float)
  depth_topic     (string, '/aurora/depth/image_raw')
  yolo_topic      (string, '/yolo_detections')
  min_confidence  (float, 0.3)  最低置信度
  depth_sample_r  (int, 5)      深度采样半径(px)
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from std_srvs.srv import SetBool

import numpy as np

# ai_msgs 只在 RDK X5 上可用; x86 测试时跳过
try:
    from ai_msgs.msg import PerceptionTargets
    HAS_AI_MSGS = True
except ImportError:
    HAS_AI_MSGS = False


class YoloConeBridge(Node):
    def __init__(self):
        super().__init__('yolo_cone_bridge')

        # 相机内参 (Aurora930 RGB, 640x400, 见 log)
        self.fx = float(self.declare_parameter('camera_fx', 416.49).value)
        self.fy = float(self.declare_parameter('camera_fy', 417.383).value)
        self.cx = float(self.declare_parameter('camera_cx', 316.832).value)
        self.cy = float(self.declare_parameter('camera_cy', 188.753).value)
        self.min_conf = float(self.declare_parameter('min_confidence', 0.25).value)
        self.depth_r = int(self.declare_parameter('depth_sample_r', 5).value)
        self.timeout_s = float(self.declare_parameter('detection_timeout_s', 1.0).value)

        depth_topic = self.declare_parameter('depth_topic', '/aurora/depth/image_raw').value
        yolo_topic = self.declare_parameter('yolo_topic', '/yolo_detections').value

        # 深度图缓存 (最新一帧)
        self._depth_img: np.ndarray | None = None
        self._depth_stamp: float = 0.0

        # YOLO 检测缓存
        self._latest_detections: list[dict] = []
        self._detection_stamp: float = 0.0

        # 订阅
        self._depth_sub = self.create_subscription(
            Image, depth_topic, self._on_depth,
            qos_profile_sensor_data)
        if HAS_AI_MSGS:
            self._yolo_sub = self.create_subscription(
                PerceptionTargets, yolo_topic, self._on_yolo, 10)
        else:
            self.get_logger().warn('ai_msgs 不可用, YOLO 订阅未创建 (x86测试环境?)')

        # 发布 (与 lidar_cone_detector 相同话题)
        self._pub_det = self.create_publisher(Bool, '/cone_detected', 10)
        self._pub_dist = self.create_publisher(Float32, '/cone_distance', 10)
        self._pub_bearing = self.create_publisher(Float32, '/cone_bearing', 10)

        # ── 门控: Mission Manager 只在段2开启 ────────────────
        self._enabled = True
        self._enable_srv = self.create_service(SetBool, '~/enable', self._on_enable)

        # 定时发布 (30Hz, 2026-07-22 提速): 从最新检测计算并发布 cone 接口
        self._timer = self.create_timer(1.0 / 30.0, self._publish_cone)

        self.get_logger().info(
            f'yolo_cone_bridge 就绪: fx={self.fx:.1f} cx={self.cx:.1f} '
            f'min_conf={self.min_conf} depth_r={self.depth_r} '
            f'timeout={self.timeout_s}s @ 30Hz')

    # ── 门控回调 ─────────────────────────────────────────────
    def _on_enable(self, req, resp):
        self._enabled = req.data
        resp.success = True
        resp.message = f"yolo_cone_bridge {'enabled' if self._enabled else 'disabled'}"
        return resp

    def _on_depth(self, msg: Image):
        """缓存最新深度图 (16UC1, mm)."""
        try:
            raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                msg.height, msg.width)
            self._depth_img = raw.copy()
            self._depth_stamp = time.time()
        except Exception as e:
            self.get_logger().warn(f'深度图解析失败: {e}')

    def _on_yolo(self, msg):
        """缓存最新 YOLO 检测."""
        dets = []
        for target in msg.targets:
            for roi in target.rois:
                dets.append({
                    'x': roi.rect.x_offset + roi.rect.width / 2,
                    'y': roi.rect.y_offset + roi.rect.height / 2,
                    'w': roi.rect.width,
                    'h': roi.rect.height,
                    'conf': roi.confidence,
                    'cls': target.type,
                })
        self._latest_detections = dets
        self._detection_stamp = time.time()

    def _publish_cone(self):
        # 禁用时发无锥桶状态
        if not self._enabled:
            self._pub_det.publish(Bool(data=False))
            self._pub_dist.publish(Float32(data=float('nan')))
            self._pub_bearing.publish(Float32(data=float('nan')))
            return

        now = time.time()

        # 检测超时 → 无锥桶
        if now - self._detection_stamp > self.timeout_s:
            self._pub_det.publish(Bool(data=False))
            self._pub_dist.publish(Float32(data=float('nan')))
            self._pub_bearing.publish(Float32(data=float('nan')))
            return

        # 过滤低置信度, 取最近距离
        best = None
        best_dist = float('inf')
        for d in self._latest_detections:
            if d['conf'] < self.min_conf:
                continue
            dist = self._get_depth(d['x'], d['y'])
            if dist is not None and dist < best_dist:
                best_dist = dist
                best = d

        if best is None:
            self._pub_det.publish(Bool(data=False))
            self._pub_dist.publish(Float32(data=float('nan')))
            self._pub_bearing.publish(Float32(data=float('nan')))
            return

        # 方位角: atan2(bx - cx, fx)  → 右负左正 (同 lidar_cone_detector)
        bearing = math.atan2(best['x'] - self.cx, self.fx)

        dist = best_dist
        if dist is not None:
            dist = dist  # 保持 float

        self._pub_det.publish(Bool(data=True))
        self._pub_dist.publish(Float32(data=float(dist) if dist is not None else float('nan')))
        self._pub_bearing.publish(Float32(data=float(bearing)))

    def _get_depth(self, cx: float, cy: float) -> float | None:
        """取检测框中心周围深度中值 (m)."""
        if self._depth_img is None:
            return None
        h, w = self._depth_img.shape
        x0 = max(0, int(cx) - self.depth_r)
        x1 = min(w, int(cx) + self.depth_r + 1)
        y0 = max(0, int(cy) - self.depth_r)
        y1 = min(h, int(cy) + self.depth_r + 1)
        roi = self._depth_img[y0:y1, x0:x1]
        valid = roi[roi > 0]
        if valid.size == 0:
            return None
        return float(np.median(valid)) / 1000.0  # mm → m


def main():
    rclpy.init()
    node = YoloConeBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        pass  # shutdown race: context already destroyed
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except RuntimeError:
            pass


if __name__ == '__main__':
    main()
