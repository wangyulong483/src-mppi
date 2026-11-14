#!/usr/bin/env python3
"""
path_recorder_node.py -- 路径打点 (段2 惯导功能包第一个节点)
================================================================
用法场景:
  手机蓝牙遥控小车(STM32 底盘)沿黄色环路跑一圈(顺/逆时针), 本节点在后台
  记录轨迹, 存成 CSV, 供后续回放节点使用。

定位来源 (纯惯导, 不用雷达):
  直接订阅 EKF 融合输出 /odom_combined (IMU + 编码器, odom 系)。
  不依赖 AMCL / 激光 / 地图 —— 雷达定位太慢, 这里走纯航迹推算。
  轨迹是相对开机点 (odom 原点) 的; 回放时车也要从同一物理点、odom 归零开始。

触发方式:
  按距离自动。每当离上一个记录点的平移 >= record_distance(默认 0.1m)就记一点。
  第一点在开始录制时立即记录。

输出 CSV 列:  index,x,y,yaw,v
  x,y   : odom 系平移 (m)
  yaw   : 航向 (rad, 由 odom 四元数解算)
  v     : 前进线速度 (m/s, 取自 /odom_combined 的 twist.linear.x)

控制服务 (std_srvs/Trigger):
  ~/start_recording : 新开一个 CSV 文件并开始录制(立即记第一点)
  ~/stop_recording  : 停止录制, 刷盘, 打印文件路径 / 点数 / 闭环误差

参数:
  odom_topic       (string, '/odom_combined') EKF 融合里程计 (位姿+速度都取自它)
  record_distance  (double, 0.1)  相邻记录点最小平移间隔 (m)
  rate             (double, 20.0) 采样频率 (Hz)
  output_dir       (string, '~/origincar_paths') CSV 输出目录
  auto_start       (bool,   True) 启动即开始录制 (False 则等 start 服务)
"""

import os
import csv
import math
from datetime import datetime

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

import tf2_ros
from rclpy.time import Time
from rclpy.duration import Duration


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """四元数 -> yaw (rad). 2D 下 x,y≈0, 但按通式算更稳。"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PathRecorder(Node):
    def __init__(self):
        super().__init__('path_recorder')

        # ── 参数 ─────────────────────────────────────────────
        self.odom_topic = self.declare_parameter('odom_topic', '/odom_combined').value
        self.record_distance = float(self.declare_parameter('record_distance', 0.1).value)
        self.rate = float(self.declare_parameter('rate', 20.0).value)
        self.output_dir = os.path.expanduser(
            self.declare_parameter('output_dir', '~/origincar_paths').value)
        self.auto_start = bool(self.declare_parameter('auto_start', True).value)

        # ── 录制起点 map 位姿捕获 (供 path_follower 的 map 绝对锚定) ──
        # 录制时若 AMCL/定位在线, 抓 map→base 存进 CSV 边车 <csv>.origin,
        # 记录环道起点的绝对 map 位姿 P_rec_map。回放联合跑时据此把环道锚到真实黄环,
        # 与 Nav2 停靠位姿无关。无 TF(纯离线录制)则跳过, 不影响 odom 锚定。
        self.capture_map_origin = bool(
            self.declare_parameter('capture_map_origin', True).value)
        self.map_frame = str(self.declare_parameter('map_frame', 'map').value)
        self.base_frame = str(self.declare_parameter('base_frame', 'base_footprint').value)
        self._map_origin = None       # (x, y, yaw) 录制起点 map 位姿, 或 None
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── odom 缓存 (位姿 + 速度, 均来自 EKF /odom_combined) ──
        # 默认 QoS (RELIABLE, KEEP_LAST 10), 与 robot_localization ekf_node 的
        # 发布者一致 —— 用 BEST_EFFORT 会因 QoS 不兼容收不到消息。
        self._odom = None            # (x, y, yaw, v) 或 None
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)

        # ── 录制状态 ─────────────────────────────────────────
        self._recording = False
        self._csv_file = None
        self._csv_writer = None
        self._csv_path = None
        self._count = 0
        self._path_length = 0.0
        self._last_xy = None        # 上一个"记录点"的 (x, y)
        self._first_xy = None       # 第一个记录点, 用于闭环误差
        self._warned_odom = False

        # ── 服务 ─────────────────────────────────────────────
        self.create_service(Trigger, '~/start_recording', self._srv_start)
        self.create_service(Trigger, '~/stop_recording', self._srv_stop)
        # 多圈便利: 一键"收上一圈 + 开下一圈"(原子), 连录多圈不用重启节点。
        # 每圈仍存独立 path_<stamp>.csv, 供 trajectory_pipeline 批量做中值中心线。
        self.create_service(Trigger, '~/next_lap', self._srv_next_lap)
        self._lap_count = 0

        # ── 定时器 ───────────────────────────────────────────
        self._timer = self.create_timer(1.0 / max(self.rate, 1.0), self._tick)

        self.get_logger().info(
            f"path_recorder 就绪 (纯惯导, 源={self.odom_topic}, odom 系), "
            f"每 {self.record_distance:.3f}m 记一点, 输出目录 {self.output_dir}")

        if self.auto_start:
            self._start_new_file()

    # ── odom 回调: 缓存位姿 + 前进速度 ────────────────────────
    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        v = msg.twist.twist.linear.x
        self._odom = (p.x, p.y, yaw, v)

    # ── 服务实现 ─────────────────────────────────────────────
    def _srv_start(self, request, response):
        if self._recording:
            response.success = False
            response.message = f"已在录制中: {self._csv_path}"
            return response
        self._start_new_file()
        response.success = True
        response.message = f"开始录制 -> {self._csv_path}"
        return response

    def _srv_stop(self, request, response):
        if not self._recording:
            response.success = False
            response.message = "当前未在录制"
            return response
        path, count, length, close_err = self._stop_and_close()
        response.success = True
        response.message = (f"已保存 {count} 点, 路径长 {length:.2f}m, "
                            f"闭环误差 {close_err:.3f}m -> {path}")
        return response

    def _srv_next_lap(self, request, response):
        """收上一圈(若在录)并立即开新一圈, 用于连录多圈。"""
        prev = None
        if self._recording:
            prev, count, length, close_err = self._stop_and_close()
        self._start_new_file()
        self._lap_count += 1
        response.success = True
        response.message = (
            f"第 {self._lap_count} 圈开始 -> {self._csv_path}" +
            (f"; 上一圈已存 {prev}" if prev else ""))
        return response

    # ── 录制控制 ─────────────────────────────────────────────
    def _start_new_file(self):
        os.makedirs(self.output_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._csv_path = os.path.join(self.output_dir, f"path_{stamp}.csv")
        # next_lap 连录时同一秒可能撞名, 加后缀保唯一(不丢圈)
        dup = 2
        while os.path.exists(self._csv_path):
            self._csv_path = os.path.join(self.output_dir, f"path_{stamp}_{dup}.csv")
            dup += 1
        self._csv_file = open(self._csv_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(['index', 'x', 'y', 'yaw', 'v'])
        self._csv_file.flush()
        self._count = 0
        self._path_length = 0.0
        self._last_xy = None
        self._first_xy = None
        self._map_origin = None
        self._recording = True
        self.get_logger().info(f"▶ 开始录制: {self._csv_path}")

    def _stop_and_close(self):
        self._recording = False
        close_err = 0.0
        if self._first_xy is not None and self._last_xy is not None:
            close_err = math.hypot(self._last_xy[0] - self._first_xy[0],
                                   self._last_xy[1] - self._first_xy[1])
        path, count, length = self._csv_path, self._count, self._path_length
        if self._csv_file is not None:
            self._csv_file.flush()
            self._csv_file.close()
            self._csv_file = None
            self._csv_writer = None
        self._write_origin_sidecar(path)     # 写 <csv>.origin (若已抓到 map 起点)
        # 平均点距 = 路径长 / 点间隔数; 应≈record_distance, 用于核对采样密度(窄道建议≤0.05m)。
        avg_gap = length / (count - 1) if count > 1 else 0.0
        self.get_logger().info(
            f"■ 停止录制: {count} 点, 路径长 {length:.2f}m, 平均点距 {avg_gap:.3f}m, "
            f"闭环误差 {close_err:.3f}m, 文件 {path}")
        return path, count, length, close_err

    # ── 主循环: 采样最新 odom, 按距离记录 ────────────────────
    def _tick(self):
        if not self._recording:
            return
        if self._odom is None:
            if not self._warned_odom:
                self.get_logger().warn(
                    f"暂无 {self.odom_topic} (底盘/EKF 是否在线?), 等待中...")
                self._warned_odom = True
            return
        self._warned_odom = False

        x, y, yaw, _ = self._odom

        if self._last_xy is None:
            self._record_point(x, y, yaw)      # 第一点立即记
            return

        d = math.hypot(x - self._last_xy[0], y - self._last_xy[1])
        if d >= self.record_distance:
            self._path_length += d
            self._record_point(x, y, yaw)

    def _record_point(self, x, y, yaw):
        v = self._odom[3] if self._odom is not None else 0.0
        self._csv_writer.writerow(
            [self._count, f"{x:.4f}", f"{y:.4f}", f"{yaw:.4f}", f"{v:.4f}"])
        self._csv_file.flush()          # 逐点刷盘, 崩溃/Ctrl+C 不丢数据
        if self._first_xy is None:
            self._first_xy = (x, y)
            self._capture_map_origin()  # 起点抓 map 位姿 → 边车 .origin
        self._last_xy = (x, y)
        self._count += 1
        if self._count % 20 == 0:
            self.get_logger().info(
                f"已记录 {self._count} 点, 路径长 {self._path_length:.2f}m")

    def _capture_map_origin(self):
        """录制起点抓 map→base 位姿, 存 self._map_origin=(x,y,yaw)。
        无 TF(纯离线录制, AMCL 未开)则留 None, 不写边车, 不影响 odom 锚定。"""
        if not self.capture_map_origin:
            return
        try:
            tf = self._tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time(),
                timeout=Duration(seconds=1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException, tf2_ros.TransformException) as e:
            self.get_logger().warn(
                f"未取到 {self.map_frame}→{self.base_frame} (定位未开?), "
                f"不写 .origin 边车(回放将用 odom 锚定): {e}")
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self._map_origin = (t.x, t.y, yaw)
        self.get_logger().info(
            f"已抓录制起点 map 位姿: ({t.x:.3f}, {t.y:.3f}, {yaw:.3f}rad)")

    def _write_origin_sidecar(self, csv_path):
        """把录制起点 map 位姿写到 <csv>.origin, 供 path_follower 的 map 绝对锚定。"""
        if self._map_origin is None or not csv_path:
            return
        mx, my, myaw = self._map_origin
        sidecar = csv_path + '.origin'
        try:
            with open(sidecar, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['map_x', 'map_y', 'map_yaw'])
                w.writerow([f"{mx:.4f}", f"{my:.4f}", f"{myaw:.4f}"])
            self.get_logger().info(f"已写录制起点 map 位姿边车: {sidecar}")
        except OSError as e:
            self.get_logger().warn(f"写 {sidecar} 失败: {e}")

    def destroy_node(self):
        if self._recording:
            self._stop_and_close()
        super().destroy_node()


def main():
    rclpy.init()
    node = PathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except RuntimeError:
            pass


if __name__ == '__main__':
    main()
