#!/usr/bin/env python3
"""
odom → TF 桥接 (纯轮式里程计, 参考 project_help origincar_bringup/src/odom2tf.cpp)

订阅 /odom (frame_id=odom, child_frame_id=base_footprint),
将 pose 转成 odom→base_footprint 的 Transform 广播到 /tf。

纯轮式方案: 无 EKF 融合, 里程计航向漂移靠 AMCL 激光匹配纠正。
室内有墙场景完全够用, 比 EKF+IMU 少一个标定变量。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class Odom2TF(Node):
    def __init__(self):
        super().__init__('odom2tf')
        qos = QoSProfile(depth=10,
                         durability=QoSDurabilityPolicy.VOLATILE,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(
            Odometry, 'odom', self.callback, qos)
        self.broadcaster = TransformBroadcaster(self)

    def callback(self, msg: Odometry):
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(t)


def main():
    rclpy.init()
    rclpy.spin(Odom2TF())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
