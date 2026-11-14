#!/usr/bin/env python3
"""
origincar_compition 竞赛总启动。

一条命令拉起全部子系统并自动执行任务状态机:
  t=0s  导航栈 (origincar_nav/navigation.launch.py)
        └─ 底盘(base_serial) + 雷达(Lslidar N10) + urdf/odom TF + nav2 + AMCL + RViz
  t=1s  相机: Aurora930 深度相机驱动 + base_link→depth_camera_link 静态 TF
  t=2s  扫码: qr_decoder(C++/WeChatQRCode) + qr_decision(方向决策, 发 /qr_event)
  t=12s 状态机: mission_state_machine (等 nav2 就绪后自动跑任务)

任务流程 (见 mission_state_machine):
  起点初始化位姿 → 去任务发布点(途中监控 /qr_event + 连线进度) →
  (扫到码且过半 立即改道 / 或 到达任务点) → 硬编码倒车右转复位车头朝向 →
  Nav2 前往黄色通道入口(到达朝向=起点) → 完成。

用法:
  ros2 launch origincar_compition competition.launch.py map:=/path/to/hall_map.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_dir = get_package_share_directory('origincar_nav')
    comp_dir = get_package_share_directory('origincar_compition')

    map_yaml = LaunchConfiguration('map',
                                   default=os.path.join(nav_dir, 'maps', 'room.yaml'))
    use_keepout = LaunchConfiguration('use_keepout', default='true')

    mission_params = os.path.join(comp_dir, 'config', 'mission_params.yaml')

    # === 深度相机安装位姿 (base_link → depth_camera_link) ===
    depth_cam_x = LaunchConfiguration('depth_cam_x', default='0.14')
    depth_cam_y = LaunchConfiguration('depth_cam_y', default='0.0')
    depth_cam_z = LaunchConfiguration('depth_cam_z', default='0.05')
    depth_cam_roll = LaunchConfiguration('depth_cam_roll', default='0.0')
    depth_cam_pitch = LaunchConfiguration('depth_cam_pitch', default='0.0')
    depth_cam_yaw = LaunchConfiguration('depth_cam_yaw', default='0.0')

    # ---- t=0s: 导航栈 (底盘 + 雷达 + nav2 + AMCL + RViz) ----
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_dir, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'map': map_yaml,
            'use_keepout': use_keepout,
        }.items(),
    )

    # ---- t=1s: Aurora930 深度相机驱动 + 静态 TF ----
    aurora_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('deptrum-ros-driver-aurora930'),
                'launch', 'aurora930_launch.py')
        ),
        launch_arguments={
            'rgb_enable': 'true',
            'depth_enable': 'true',
            'ir_enable': 'true',        # IR=深度传感器, 必须开才有 /aurora/depth/image_raw
            'point_cloud_enable': 'true',
            'rgb_fps': '15',
            'ir_fps': '15',
            'heart_enable': 'false',    # SDK v1.1.22 与固件 v1.7.2 心跳协议不兼容
            'stream_sdk_log_enable': 'false',
        }.items()
    )

    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='depth_camera_tf',
        arguments=[
            '--x', depth_cam_x, '--y', depth_cam_y, '--z', depth_cam_z,
            '--roll', depth_cam_roll, '--pitch', depth_cam_pitch, '--yaw', depth_cam_yaw,
            '--frame-id', 'base_link', '--child-frame-id', 'depth_camera_link',
        ],
        output='screen',
    )

    # ---- t=2s: 扫码 (qr_decoder + qr_decision, 均在 origincar_qr 包) ----
    qr_decoder_node = Node(
        package='origincar_qr',
        executable='qr_decoder',
        name='qr_decoder_zbar',
        output='screen',
        parameters=[{
            'rgb_topic': '/aurora/rgb/image_raw',
            'depth_topic': '/aurora/depth/image_raw',
            'enable_depth_filter': True,
            'depth_min_mm': 50,
            'depth_max_mm': 4000,
            'model_dir': '/usr/share/wechat_qrcode',
        }],
        arguments=['--ros-args', '--log-level', 'info'],
    )
    qr_decision_node = Node(
        package='origincar_qr',
        executable='qr_decision',
        name='qr_decision',
        output='screen',
        parameters=[{'cooldown_sec': 3.0}],
    )

    # ---- t=12s: 任务状态机 ----
    mission_node = Node(
        package='origincar_compition',
        executable='mission_state_machine',
        name='mission_state_machine',
        output='screen',
        parameters=[mission_params],
    )

    return LaunchDescription([
        DeclareLaunchArgument('map',
                              default_value=os.path.join(nav_dir, 'maps', 'room.yaml'),
                              description='大厅地图 yaml (与建图一致)'),
        DeclareLaunchArgument('use_keepout', default_value='true',
                              description='启用 keepout 禁区掩膜'),
        DeclareLaunchArgument('depth_cam_x', default_value='0.14'),
        DeclareLaunchArgument('depth_cam_y', default_value='0.0'),
        DeclareLaunchArgument('depth_cam_z', default_value='0.05'),
        DeclareLaunchArgument('depth_cam_roll', default_value='0.0'),
        DeclareLaunchArgument('depth_cam_pitch', default_value='0.0'),
        DeclareLaunchArgument('depth_cam_yaw', default_value='0.0'),

        TimerAction(period=0.0, actions=[navigation]),
        TimerAction(period=1.0, actions=[aurora_node, camera_tf]),
        TimerAction(period=2.0, actions=[qr_decoder_node, qr_decision_node]),
        TimerAction(period=12.0, actions=[mission_node]),
    ])
