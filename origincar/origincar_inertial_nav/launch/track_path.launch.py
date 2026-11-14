#!/usr/bin/env python3
"""
track_path.launch.py -- 一键实时循迹: 底盘惯导 + 纯跟踪追踪 (差速模式, 纯惯导)

和 record_path.launch.py 配套: record 录一条 CSV 参考轨迹, 这里加载它,
用惯导实时闭环跟踪(每拍按当前 odom 位姿纠偏), 自动沿轨迹跑一圈。

启动链 (TimerAction 错峰, 防 EKF 未就绪):
  t=0s  底盘 + 惯导 + TF   (origincar_base/origincar_bringup.launch.py, akmcar:=false)
          差速模式: origincar_base 直接订阅 /cmd_vel(Twist), 把 (v, ω) 下发 STM32,
          不启动阿克曼转换链。
  t=5s  path_follower       (加载最新 CSV; 默认 auto_start=false, 等 start 服务发车)

★ 差速模式: path_follower 发 Twist(linear.x=v, angular.z=ω) 到 /cmd_vel 即可,
  无需 cmd_vel_to_ackermann_drive。

一键流程:
  1) 把车摆到"和录制时完全相同的物理起点"(黄环入口), 重新开机 → odom 归零
  2) ros2 launch origincar_inertial_nav track_path.launch.py
     (默认不自动发车, 车不会乱动; 想起来就跑加 auto_start:=true)
  3) 发车: 另开终端
       ros2 service call /path_follower/start_following std_srvs/srv/Trigger
     车实时跟踪 CSV 轨迹跑一圈, 到末点自动停车。
  4) 急停: ros2 service call /path_follower/stop_following std_srvs/srv/Trigger
     (或直接 Ctrl+C, 节点退出会发零速)

常用可选参数:
  auto_start:=true            起来 5s+3s 后自动发车(危险, 确认车已摆好再用)
  csv_path:=/abs/path.csv     指定追踪哪条 CSV (默认取 path_dir 里最新的)
  path_dir:=<dir>             CSV 目录 (默认 ~/origincar_paths)
  max_speed:=0.4              直道最大线速度 (m/s)
  max_omega:=2.0              最大角速度 (rad/s), 差速转弯上限
  lat_acc_max:=0.5            弯道侧向加速度上限 (越大弯道越快越飘)
  lookahead_min:=0.3          前视距离下限 (越大越平滑但切内道)
  lookahead_curv_ratio:=0.35  曲率自适应前视: Ld≤ratio×本地转弯半径, 弯道自动缩短前视消切内弯
                              (越小越贴线但越易抖; 0=关退回纯速度前视)
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
    auto_start = LaunchConfiguration('auto_start', default='false')
    csv_path = LaunchConfiguration('csv_path', default='')
    path_dir = LaunchConfiguration('path_dir', default='~/origincar_paths')
    max_speed = LaunchConfiguration('max_speed', default='0.45')
    min_speed = LaunchConfiguration('min_speed', default='0.12')
    max_omega = LaunchConfiguration('max_omega', default='3.00')
    lat_acc_max = LaunchConfiguration('lat_acc_max', default='0.80')
    # 窄通道(~50cm)+小半径弯: 前视压小+曲率自适应+强反馈, 贴线不振荡 (2026-07-22仿真最优)
    lookahead_min = LaunchConfiguration('lookahead_min', default='0.25')
    lookahead_max = LaunchConfiguration('lookahead_max', default='0.60')
    lookahead_k = LaunchConfiguration('lookahead_k', default='0.40')
    # 曲率自适应前视: 进弯 Ld 不超过 ratio×本地转弯半径, 弯道自动缩短前视。0=关
    lookahead_curv_ratio = LaunchConfiguration('lookahead_curv_ratio', default='0.15')
    goal_tolerance = LaunchConfiguration('goal_tolerance', default='0.15')
    # 抗漂移强化增益 (置 0 退化回原纯跟踪)
    curvature_ff_gain = LaunchConfiguration('curvature_ff_gain', default='0.35')
    cross_track_gain = LaunchConfiguration('cross_track_gain', default='1.8')
    heading_gain = LaunchConfiguration('heading_gain', default='1.0')

    # ── 1) 底盘 + 惯导 + TF (差速模式 akmcar:=false) ─────────
    chassis_inertial = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(base_dir, 'launch', 'origincar_bringup.launch.py')),
        launch_arguments={'akmcar': 'false'}.items(),
    )

    # ── 2) 纯跟踪实时循迹 ───────────────────────────────────
    follower = Node(
        package='origincar_inertial_nav',
        executable='path_follower',
        name='path_follower',
        output='screen',
        parameters=[{
            'odom_topic': '/odom_combined',
            'cmd_vel_topic': '/cmd_vel',
            'csv_path': csv_path,
            'path_dir': path_dir,
            'rate': 20.0,
            'max_speed': max_speed,
            'min_speed': min_speed,
            'max_omega': max_omega,
            'lat_acc_max': lat_acc_max,
            'lookahead_min': lookahead_min,
            'lookahead_max': lookahead_max,
            'lookahead_k': lookahead_k,
            'lookahead_curv_ratio': lookahead_curv_ratio,
            'curvature_ff_gain': curvature_ff_gain,
            'cross_track_gain': cross_track_gain,
            'heading_gain': heading_gain,
            'goal_tolerance': goal_tolerance,
            'search_window': 40,
            'auto_start': auto_start,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('auto_start', default_value='false',
                              description='true=起来后自动发车(危险); false=等 start 服务'),
        DeclareLaunchArgument('csv_path', default_value='',
                              description='指定追踪的 CSV; 空则取 path_dir 里最新的'),
        DeclareLaunchArgument('path_dir', default_value='~/origincar_paths',
                              description='CSV 目录'),
        DeclareLaunchArgument('max_speed', default_value='0.45',
                              description='直道最大线速度 (m/s)'),
        DeclareLaunchArgument('min_speed', default_value='0.12',
                              description='最小线速度 (m/s)'),
        DeclareLaunchArgument('max_omega', default_value='3.00',
                              description='最大角速度 (rad/s)'),
        DeclareLaunchArgument('lat_acc_max', default_value='0.80',
                              description='弯道侧向加速度上限 (m/s^2); 0.60→0.80 弯中不减速太猛'),
        DeclareLaunchArgument('lookahead_min', default_value='0.25',
                              description='前视距离下限 (m); 实车防微摆, 0.20→0.25'),
        DeclareLaunchArgument('lookahead_max', default_value='0.60',
                              description='前视距离上限 (m); 实车防微摆, 0.50→0.60'),
        DeclareLaunchArgument('lookahead_k', default_value='0.40',
                              description='前视随速增益 Ld=min+k·v'),
        DeclareLaunchArgument('lookahead_curv_ratio', default_value='0.15',
                              description='曲率自适应前视: Ld<=ratio*本地转弯半径, 消切内弯 (0=关)'),
        DeclareLaunchArgument('goal_tolerance', default_value='0.15',
                              description='到末点多近算完成 (m)'),
        DeclareLaunchArgument('curvature_ff_gain', default_value='0.35',
                              description='曲率前馈增益: 按录制曲率提前转弯消切弯 (0=关)'),
        DeclareLaunchArgument('cross_track_gain', default_value='1.8',
                              description='横向偏差(CTE)拉回增益 rad/s per m; 仿真最优 (0=关)'),
        DeclareLaunchArgument('heading_gain', default_value='1.0',
                              description='航向偏差拉回增益 rad/s per rad; 仿真最优 (0=关)'),

        TimerAction(period=0.0, actions=[chassis_inertial]),
        TimerAction(period=5.0, actions=[follower]),
    ])
