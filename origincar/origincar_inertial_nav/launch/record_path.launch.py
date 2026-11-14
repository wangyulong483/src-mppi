#!/usr/bin/env python3
"""
record_path.launch.py -- 一键启动: 底盘惯导 + 路径打点 (纯惯导, 不用雷达)

启动链 (TimerAction 错峰, 防 EKF 未就绪):
  t=0s  底盘 + 惯导 + TF   (origincar_base/origincar_bringup.launch.py:
                            base_serial + bmi088_bridge + madgwick + EKF +
                            robot_state_publisher(origincar_slam.urdf))
  t=3s  path_recorder      (订阅 /odom_combined, auto_start 默认 true → 起来就录)

纯航迹推算: 只用 IMU(BMI088+Madgwick) + 编码器经 EKF 融合出 /odom_combined,
不启动雷达 / AMCL / 地图 —— 雷达定位太慢。轨迹记录在 odom 系(相对开机点)。
录制时靠手机蓝牙遥控 STM32 底盘跑环路, 不需要 ROS 发 /cmd_vel。

一键流程:ros2 launch origincar_inertial_nav record_path.launch.py
  1) 
     (车摆在环路起点/黄环入口, 开机即 odom 原点, 起来就开始录)
  2) 手机蓝牙遥控车沿黄环跑一圈回到入口
  3) Ctrl+C → 节点收尾时刷盘并打印 CSV 路径 / 点数 / 闭环误差
     (数据本来就逐点刷盘, Ctrl+C 不会丢)

常用可选参数:
  auto_start:=false      改成手动 start/stop 服务圈定一圈 (默认 true=起来就录)
  record_distance:=0.05  打点间隔(m)
  output_dir:=<dir>      CSV 输出目录 (默认 ~/origincar_paths)
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
    base_dir = get_package_share_directory('origincar_base')

    # ── 启动参数 ─────────────────────────────────────────────
    # 默认 0.05m: 窄通道(≈50cm)+小半径弯(R≈0.57m)下, 加密打点让参考线曲率更清晰,
    # 供 path_follower 的曲率自适应前视准确地在入弯点缩短 Ld(消切内弯)。
    record_distance = LaunchConfiguration('record_distance', default='0.05')
    auto_start = LaunchConfiguration('auto_start', default='true')
    output_dir = LaunchConfiguration('output_dir', default='~/origincar_paths')

    # ── 1) 底盘 + 惯导 + TF ─────────────────────────────────
    # 含 base_serial / bmi088_bridge / madgwick / EKF / robot_state_publisher(slam.urdf)。
    # EKF 发 /odom_combined + odom->base_footprint。不含雷达。
    chassis_inertial = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(base_dir, 'launch', 'origincar_bringup.launch.py')),
    )

    # ── 2) 路径打点 ─────────────────────────────────────────
    recorder = Node(
        package='origincar_inertial_nav',
        executable='path_recorder',
        name='path_recorder',
        output='screen',
        parameters=[{
            'odom_topic': '/odom_combined',
            'record_distance': record_distance,
            'rate': 20.0,
            'output_dir': output_dir,
            'auto_start': auto_start,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('record_distance', default_value='0.05',
                              description='相邻记录点最小平移间隔 (m); 窄道/小弯建议 0.05'),
        DeclareLaunchArgument('auto_start', default_value='true',
                              description='true=启动即录(Ctrl+C 保存); false=手动 start/stop 服务'),
        DeclareLaunchArgument('output_dir', default_value='~/origincar_paths',
                              description='CSV 输出目录'),

        TimerAction(period=0.0, actions=[chassis_inertial]),
        TimerAction(period=3.0, actions=[recorder]),
    ])
