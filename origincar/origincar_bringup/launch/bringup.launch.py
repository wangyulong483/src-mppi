#!/usr/bin/env python3
"""
origincar 导航/SLAM 底层总启动 (纯轮式里程计, 差速模式, 参考 project_help)。

组成:
  1. base_serial (akmcar=false): 底盘串口, 直接订阅 /cmd_vel(Twist),
     STM32 固件内部处理阿克曼运动学转换, 发布 /odom(frame=odom, child=base_footprint)。
  2. Lslidar N10 驱动: 发布 /scan (frame_id=laser_link)。
  3. urdf2tf: robot_state_publisher 发布 base_footprint->base_link->laser_link。
  4. odom2tf: 订阅 /odom, 将 pose 转成 odom->base_footprint TF 广播。

用法: ros2 launch origincar_bringup bringup.launch.py
"""

import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    base_dir = get_package_share_directory('origincar_base')
    bringup_dir = get_package_share_directory('origincar_bringup')
    lslidar_dir = get_package_share_directory('lslidar_driver')

    lslidar_params = os.path.join(lslidar_dir, 'params', 'lsx10.yaml')

    # 1. 底盘 (差速模式: Nav2 /cmd_vel -> STM32 固件内部阿克曼转换)
    base_serial = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(base_dir, 'launch', 'base_serial.launch.py')),
        launch_arguments={'akmcar': 'false'}.items(),
    )

    # 2. Lslidar N10 驱动 (只起驱动节点, 不带驱动自带的 rviz; frame_id=laser_link, /scan)
    lslidar_node = launch_ros.actions.Node(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[lslidar_params],
    )

    # 3. URDF -> TF (含 laser_link)
    urdf2tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'urdf2tf.launch.py')),
    )

    # 4. odom -> TF: 纯轮式, 无 EKF
    odom2tf = launch_ros.actions.Node(
        package='origincar_bringup',
        executable='odom2tf.py',
        name='odom2tf',
        output='screen',
    )

    return LaunchDescription([
        base_serial,
        lslidar_node,
        urdf2tf,
        odom2tf,
    ])
