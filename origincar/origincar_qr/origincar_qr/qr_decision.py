#!/usr/bin/env python3
"""
qr_decision — QR 方向决策节点 (普通 Node, 非 Lifecycle)
========================================================
订阅 /sign (qr_decoder ZBar解码结果) + /odom 位姿 → 方向判断 → /qr_event。
始终在线; lifecycle_manager 管理时只打 warn log, 不崩溃。

方向规则: 奇数→CW(顺时针), 偶数→CCW(逆时针)。
冷却: 同号3s内不重复触发。
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from std_msgs.msg import String, Int32
from nav_msgs.msg import Odometry


class QrDecisionNode(Node):
    def __init__(self):
        super().__init__("qr_decision")

        self.declare_parameter("cooldown_sec", 3.0)
        self.cooldown_sec = (
            self.get_parameter("cooldown_sec").get_parameter_value().double_value)

        self._last_qr_num = None
        self._last_trigger_time = self.get_clock().now()
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0

        # 发布 (qr_result 改由 mission_state_machine 在半场触发时发布)
        self.pub_qr_event = self.create_publisher(String, "qr_event", 10)
        sign_qos = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_sign_data = self.create_publisher(
            Int32, "/sign_switch_sign_data", sign_qos)

        # 订阅
        self.sub_sign = self.create_subscription(
            String, "/sign", self.sign_callback, 10)
        self.sub_odom = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10)

        self.get_logger().info(
            f"qr_decision ready: cooldown={self.cooldown_sec}s")

    def odom_callback(self, msg):
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._odom_yaw = math.atan2(siny, cosy)

    def sign_callback(self, msg):
        text = msg.data.strip()
        if not text:
            return

        if not text.isdigit() or len(text) < 1 or len(text) > 4:
            self.get_logger().info(
                f"[qr_decision] REJECT non-4-digit QR: '{text}' "
                f"(len={len(text)}, digit={text.isdigit()})",
                throttle_duration_sec=2.0)
            return

        num = int(text)
        if not (1 <= num <= 9999):
            self.get_logger().debug(f"QR number out of valid range [1,9999]: {num}")
            return

        now = self.get_clock().now()
        if num == self._last_qr_num:
            elapsed = (now - self._last_trigger_time).nanoseconds / 1e9
            if elapsed < self.cooldown_sec:
                self.get_logger().info(
                    f"[qr_decision] COOLDOWN blocked qr={num} "
                    f"elapsed={elapsed:.1f}s < {self.cooldown_sec}s",
                    throttle_duration_sec=2.0)
                return

        self._last_qr_num = num
        self._last_trigger_time = now

        direction = "顺时针" if num % 2 == 1 else "逆时针"
        dir_short = "CW" if num % 2 == 1 else "CCW"

        self.get_logger().info(
            f"[QR] {num} {dir_short} @ "
            f"(x={self._odom_x:.2f}, y={self._odom_y:.2f}, "
            f"yaw={self._odom_yaw:.2f}) -> /qr_event")

        # qr_result 改由 mission_state_machine 在过半场 startReorient() 时发布
        self.pub_qr_event.publish(
            String(data=f"{num},{direction},{self._odom_x:.2f},{self._odom_y:.2f}"))
        value = 3 if direction == "顺时针" else 4
        self.pub_sign_data.publish(Int32(data=value))


def main(args=None):
    rclpy.init(args=args)
    node = QrDecisionNode()
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


if __name__ == "__main__":
    main()
