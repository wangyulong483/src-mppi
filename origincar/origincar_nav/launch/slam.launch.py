#!/usr/bin/env python3
"""
SLAM 建图启动 (slam_toolbox 在线异步建图)。

启动链:
  t=0s  origincar_bringup: 底盘(阿克曼) + Lslidar N10(/scan) + urdf2tf(含 laser_link) + BMI088 + EKF
        → TF: odom -> base_footprint -> base_link -> laser_link
  t=4s  slam_toolbox async 建图节点 (发布 map -> odom)
  t=6s  RViz2

用法:
  ros2 launch origincar_nav slam.launch.py
  # 遥控/手推小车走遍大厅 (base akmcar=true, teleop 发 /cmd_vel 即可),
  # RViz 观察地图成形, 然后保存地图:
  ros2 run nav2_map_server map_saver_cli -f ~/dev_ws/hall_map --ros-args -p save_map_timeout:=5000
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
    bringup_dir = get_package_share_directory('origincar_bringup')

    slam_params_file = os.path.join(nav_dir, 'config', 'slam_toolbox_mapping.yaml')
    rviz_config = os.path.join(nav_dir, 'rviz', 'nav.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation clock if true'),

        # t=0s: 底盘 + 雷达 + URDF/TF + BMI088 + EKF (odom->base_footprint)
        TimerAction(period=0.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'launch', 'bringup.launch.py')),
            ),
        ]),

        # t=4s: slam_toolbox 在线异步建图 (等 TF/scan 就绪)
        TimerAction(period=4.0, actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
            ),
        ]),

        # t=6s: RViz2
        TimerAction(period=6.0, actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen',
            ),
        ]),
    ])
