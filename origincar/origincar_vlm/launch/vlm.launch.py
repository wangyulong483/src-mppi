#!/usr/bin/env python3
"""
vlm.launch.py -- VLM 图生文调试模式
=====================================
仅启动深度相机 (Aurora930 RGB) + VLM 节点 (manual 模式)。
小车不动、不启雷达、不用 Nav2。
终端按 Enter 拍照 → 云端推理 → 终端显示结果.

用法:
  ros2 launch origincar_vlm vlm.launch.py
  # API Key 已硬编码, 无需设环境变量
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    aurora_share = get_package_share_directory('deptrum-ros-driver-aurora930')

    # ── 深度相机: 只开 RGB (15fps, 640x400) ──────────────────
    aurora = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(aurora_share, 'launch', 'aurora930_launch.py')),
        launch_arguments={
            'rgb_enable': 'true',
            'depth_enable': 'false',
            'ir_enable': 'false',
            'point_cloud_enable': 'false',
            'rgb_fps': '15',
            'heart_enable': 'false',
        }.items(),
    )

    # 深度相机 TF (base_link → depth_camera_link)
    depth_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='depth_camera_tf',
        arguments=['0.14', '0', '0.05', '0', '0', '0',
                   'base_link', 'depth_camera_link'],
    )

    # ── VLM 节点: manual 模式, 按 Enter 触发 ──────────────────
    vlm = Node(
        package='origincar_vlm',
        executable='vlm_node',
        name='vlm_node',
        output='screen',
        parameters=[{
            'trigger_mode': 'manual',
            'enable_debug': True,   # 保存 /tmp/vlm_debug.jpg 方便查看
            'api_key': LaunchConfiguration('api_key'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('api_key',
            default_value='sk-ws-H.EHMYXEL.FrRF.MEQCIAwLkXMEuoaBtVf4BbPybwdZqeepKN3lmJ0svdOqa-PXAiBiDpwI-dEc81oAs_CyKc2phRc8yTJJNllB2Cq5MFmLzg',
            description='阿里云百炼 DashScope API Key'),

        aurora,
        depth_camera_tf,
        vlm,
    ])
