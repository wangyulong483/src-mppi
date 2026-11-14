#!/usr/bin/env python3
"""
record_multilap.launch.py -- 多圈连录 (给中值中心线流水线备料)
================================================================
起 path_recorder(auto_start=false, 窄道建议更密的 record_distance), 便于用
~/next_lap 服务连录多圈。每圈存独立 path_<stamp>.csv 到同一目录, 之后交给
trajectory_pipeline 做"横向中值中心线 + 通道内外分析"。

前提: 底盘 + EKF 已在别处起好, /odom_combined 在发 (同 record_only)。

用法:
  ros2 launch origincar_inertial_nav record_multilap.launch.py
  # 遥控把车摆到 P 点(odom 归零), 然后逐圈:
  ros2 service call /path_recorder/next_lap std_srvs/srv/Trigger   # 开第1圈
  #   ... 遥控跑完一圈 ...
  ros2 service call /path_recorder/next_lap std_srvs/srv/Trigger   # 收上圈+开下圈
  #   ... 重复 5~10 圈 ...
  ros2 service call /path_recorder/stop_recording std_srvs/srv/Trigger  # 收最后一圈

  # 全部录完后提炼中心线(仅 B+C 段, 输出 follower 可直接用):
  ros2 run origincar_inertial_nav trajectory_pipeline ~/origincar_paths \
       -o <ws>/config/path_ccw.csv --direction ccw --plot ccw_analysis.png
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    record_distance = LaunchConfiguration('record_distance', default='0.05')
    output_dir = LaunchConfiguration('output_dir', default='~/origincar_paths')
    odom_topic = LaunchConfiguration('odom_topic', default='/odom_combined')

    return LaunchDescription([
        DeclareLaunchArgument('record_distance', default_value='0.05',
                              description='相邻记录点最小平移间隔 (m); 窄道建议 ≤0.05'),
        DeclareLaunchArgument('output_dir', default_value='~/origincar_paths',
                              description='多圈 CSV 输出目录 (交给 trajectory_pipeline)'),
        DeclareLaunchArgument('odom_topic', default_value='/odom_combined',
                              description='EKF 融合里程计话题'),

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
                'auto_start': False,     # 等 next_lap 发第一圈, 更安全
            }],
        ),
    ])
