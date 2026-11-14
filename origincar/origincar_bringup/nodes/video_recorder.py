#!/usr/bin/env python3
"""
video_recorder.py — 深度相机视频录制节点

订阅 Aurora930 深度相机的 RGB 原始图像 (/aurora/rgb/image_raw, bgr8)，
使用 OpenCV VideoWriter 录制为 MP4 视频文件。

节点启动即自动录制，Ctrl+C 停止时自动保存；
也可通过 ROS2 服务手动控制启停。
"""

import os
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool


class VideoRecorder(Node):
    """订阅图像话题并录制为 MP4 文件的 ROS2 节点。"""

    def __init__(self):
        super().__init__('video_recorder')

        # ---- 参数 ----
        self.declare_parameter('image_topic', '/aurora/rgb/image_raw')
        self.declare_parameter('output_dir', '/tmp/video_records')
        self.declare_parameter('filename_prefix', 'aurora930')
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('fourcc', 'mp4v')   # mp4v / avc1 / XVID
        self.declare_parameter('file_extension', '.mp4')

        self._image_topic = self.get_parameter('image_topic').value
        self._output_dir = self.get_parameter('output_dir').value
        self._filename_prefix = self.get_parameter('filename_prefix').value
        self._fps = self.get_parameter('fps').value
        self._fourcc_str = self.get_parameter('fourcc').value
        self._ext = self.get_parameter('file_extension').value

        # ---- 内部状态 ----
        self._bridge = CvBridge()
        self._writer = None       # cv2.VideoWriter 实例
        self._frame_count = 0
        self._current_file = ''

        # 确保输出目录存在
        os.makedirs(self._output_dir, exist_ok=True)

        # 自动开始录制：预生成文件名，VideoWriter 在首帧到达时实际创建
        ts = time.strftime('%Y%m%d_%H%M%S')
        fname = f'{self._filename_prefix}_{ts}{self._ext}'
        self._current_file = os.path.join(self._output_dir, fname)
        self._recording = True

        # ---- 订阅 (独立回调组，不受服务调用阻塞) ----
        sub_cb_group = MutuallyExclusiveCallbackGroup()
        self._sub = self.create_subscription(
            Image,
            self._image_topic,
            self._image_callback,
            10,
            callback_group=sub_cb_group,
        )

        # ---- 服务: 手动控制录制启停 (可选) ----
        self._srv = self.create_service(
            SetBool,
            '~/record',
            self._record_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.get_logger().info(
            f'▶  自动录制已就绪 → {self._current_file} '
            f'| topic={self._image_topic} | fps={self._fps}'
        )

    # ----------------------------------------------------------------
    #  图像回调
    # ----------------------------------------------------------------
    def _image_callback(self, msg: Image):
        if not self._recording:
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge 转换失败: {e}')
            return

        # 延迟初始化 VideoWriter —— 首帧到达时确定画面尺寸
        if self._writer is None:
            h, w = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*self._fourcc_str)
            self._writer = cv2.VideoWriter(
                self._current_file, fourcc, self._fps, (w, h)
            )
            if not self._writer.isOpened():
                self.get_logger().error(f'无法打开 VideoWriter: {self._current_file}')
                self._recording = False
                return
            self.get_logger().info(f'VideoWriter 已打开: {self._current_file} ({w}x{h})')

        self._writer.write(frame)
        self._frame_count += 1

    # ----------------------------------------------------------------
    #  服务回调
    # ----------------------------------------------------------------
    def _record_callback(self, req, resp):
        if req.data:
            # ---- 开始录制 ----
            if self._recording:
                resp.success = False
                resp.message = '已在录制中'
                return resp

            ts = time.strftime('%Y%m%d_%H%M%S')
            fname = f'{self._filename_prefix}_{ts}{self._ext}'
            self._current_file = os.path.join(self._output_dir, fname)
            self._recording = True
            self._frame_count = 0
            # VideoWriter 在首帧到达时创建
            self.get_logger().info(f'▶  开始录制 → {self._current_file}')
            resp.success = True
            resp.message = f'开始录制: {fname}'
        else:
            # ---- 停止录制 ----
            if not self._recording:
                resp.success = False
                resp.message = '未在录制'
                return resp

            self._recording = False
            if self._writer is not None:
                self._writer.release()
                self._writer = None
            self.get_logger().info(
                f'■ 停止录制 → {self._current_file} '
                f'(共 {self._frame_count} 帧, '
                f'{self._frame_count / self._fps:.1f}s @ {self._fps}fps)'
            )
            resp.success = True
            resp.message = (
                f'已停止: {os.path.basename(self._current_file)} '
                f'({self._frame_count} frames)'
            )
            self._current_file = ''

        return resp

    # ----------------------------------------------------------------
    #  析构
    # ----------------------------------------------------------------
    def destroy_node(self):
        self._recording = False
        if self._writer is not None:
            self._writer.release()
            self.get_logger().info(
                f'■ 录制已保存 → {self._current_file} '
                f'(共 {self._frame_count} 帧, '
                f'{self._frame_count / max(self._fps, 1):.1f}s @ {self._fps}fps)'
            )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
