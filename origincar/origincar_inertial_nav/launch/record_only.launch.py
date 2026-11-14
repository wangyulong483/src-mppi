#!/usr/bin/env python3
"""
record_only.launch.py -- 只启动打点节点 (底盘/惯导已在别处跑起来时用)

一键请用 record_path.launch.py。本文件只起 path_recorder, 适合:
  - 你已经用别的 launch 起好了底盘+EKF, /odom_combined 在发
  - 调试打点节点本身

用法:
  ros2 launch origincar_inertial_nav record_only.launch.py
  ros2 launch origincar_inertial_nav record_only.launch.py record_distance:=0.05 auto_start:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    record_distance = LaunchConfiguration('record_distance', default='0.1')
    auto_start = LaunchConfiguration('auto_start', default='true')
    output_dir = LaunchConfiguration('output_dir', default='~/origincar_paths')
    odom_topic = LaunchConfiguration('odom_topic', default='/odom_combined')

    return LaunchDescription([
        DeclareLaunchArgument('record_distance', default_value='0.1',
                              description='相邻记录点最小平移间隔 (m)'),
        DeclareLaunchArgument('auto_start', default_value='true',
                              description='true=启动即录制; false=等 start 服务'),
        DeclareLaunchArgument('output_dir', default_value='~/origincar_paths',
                              description='CSV 输出目录'),
        DeclareLaunchArgument('odom_topic', default_value='/odom_combined',
                              description='取前进速度 v 的里程计话题'),

        Node(
            package='origincar_inertial_nav',
            executable='path_recorder',
            name='path_recorder',
            output='screen',
            parameters=[{
                'odom_topic': odom_topic,
                'record_distance': record_distance,
                'rate': 20.0,
                'output_dir': output_dir,
                'auto_start': auto_start,
            }],
        ),
    ])
