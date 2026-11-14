#!/usr/bin/env python3
"""
stage2.launch.py -- 段2 一键启动: 环道循迹 + YOLO锥桶避障 (总控)

在段2 track_path 之上叠加 YOLO 锥桶避障。启动链 (TimerAction 错峰, 防 EKF 未就绪):
  t=0s  底盘 + 惯导 + TF   (origincar_base/origincar_bringup.launch.py, akmcar:=false)
          (含 robot_state_publisher, 发布 base_link->laser_link TF)
  t=0s  Lslidar N10 驱动   (lslidar_driver, 发布 /scan, frame_id=laser_link)
  t=0s  Aurora930 相机      (RGB+深度, 供 YOLO 检测)
  t=3s  YOLO BPU 推理       (origincar_yolo, /aurora/rgb/image_raw → /yolo_detections)
  t=3s  YOLO→Cone 桥接     (yolo_cone_bridge, /yolo_detections + depth → /cone_*)
  t=5s  path_follower       (循迹, cmd_vel 重映射到 /path_follower/cmd_vel, auto_start=false)
  t=7s  stage2_node         (YOLO锥桶避障状态机, /cmd_vel 唯一发布者)

方向选择:
  direction:=cw   (默认) 走顺时针, 加载 cw_csv
  direction:=ccw         走逆时针, 加载 ccw_csv

常用可选参数:
  max_speed:=0.50                循迹直道最大线速度 (m/s)
  curvature_ff_gain:=0.35       曲率前馈增益
  cone_avoid_danger_m:=0.6      锥桶全避距(m): 近于此全避让
  cone_avoid_influence_m:=1.5   锥桶起避距(m): 开始融合
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ── 方向相关参数: (name, DeclareLaunchArgument原始默认, cw默认, ccw默认) ──
# OpaqueFunction 自动按 direction 切换; 命令行显式传参仍优先
_DIRECTION_PARAMS = [
    ('curvature_ff_gain',       '0.35', '0.20', '0.35'),
    ('lookahead_k',             '0.80', '0.50', '0.80'),
    ('cone_avoid_creep_v',      '0.40', '0.40', '0.40'),
    ('cone_avoid_duration_s',   '0.60', '0.60', '0.60'),
    ('cone_avoid_max_omega',    '5.00', '5.00', '5.00'),
]


def _resolve_csv(context):
    """按 direction 选 CSV 路径, 展开 ~ 后作为 path_follower 的 csv_path。"""
    direction = context.launch_configurations.get('direction', 'cw').lower()
    cw_csv = context.launch_configurations['cw_csv']
    ccw_csv = context.launch_configurations['ccw_csv']
    csv = ccw_csv if direction == 'ccw' else cw_csv
    csv = os.path.expanduser(csv)
    if not os.path.isfile(csv):
        print(f"[stage2.launch] 警告: 找不到 {direction} 路径 CSV: {csv}")
    print(f"[stage2.launch] direction={direction}, 加载路径: {csv}")
    context.launch_configurations['resolved_csv'] = csv
    return []


def _resolve_direction_defaults(context):
    """按 direction 切换方向相关参数的默认值 (cw=实车调优值, ccw=原始默认)。
    命令行显式传参不会被覆盖。"""
    direction = context.launch_configurations.get('direction', 'cw').lower()
    for name, original, cw_val, ccw_val in _DIRECTION_PARAMS:
        current = context.launch_configurations.get(name, original)
        if current == original:  # 用户未显式传参 → 按方向自动选默认值
            val = cw_val if direction == 'cw' else ccw_val
            context.launch_configurations[name] = val
    return []


def generate_launch_description():
    base_dir = get_package_share_directory('origincar_base')
    nav_share = get_package_share_directory('origincar_inertial_nav')
    lslidar_share = get_package_share_directory('lslidar_driver')
    lslidar_params = os.path.join(lslidar_share, 'params', 'lsx10.yaml')
    aurora_share = get_package_share_directory('deptrum-ros-driver-aurora930')
    stage2_share = get_package_share_directory('origincar_stage2')
    default_cw_csv = os.path.join(nav_share, 'config', 'path_cw.csv')
    default_ccw_csv = os.path.join(nav_share, 'config', 'path_ccw.csv')

    direction = LaunchConfiguration('direction', default='cw')

    # ── 1) 底盘 + 惯导 + TF (差速模式 akmcar:=false) ─────────
    chassis_inertial = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(base_dir, 'launch', 'origincar_bringup.launch.py')),
        launch_arguments={'akmcar': 'false'}.items(),
    )

    # ── 2) Lslidar N10 驱动 (/scan, frame_id=laser_link) ────
    lslidar_node = Node(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        parameters=[lslidar_params],
    )

    # ── 3) Aurora930 相机 (RGB+深度, 供 YOLO 检测和桥接测距) ──
    aurora930 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(aurora_share, 'launch', 'aurora930_launch.py')),
        launch_arguments={
            'rgb_enable': 'true',
            'depth_enable': 'true',
            'ir_enable': 'false',
            'point_cloud_enable': 'false',
            'heart_enable': 'false',
        }.items(),
    )
    depth_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='depth_camera_tf',
        arguments=['0.14', '0', '0.05', '0', '0', '0',
                   'base_link', 'depth_camera_link'],
    )

    # ── 4) YOLO BPU 推理 (Aurora930 RGB → /yolo_detections) ──
    yolo_config = os.path.join(stage2_share, 'config', 'yolov5workconfig.json')
    yolo_node = Node(
        package='origincar_yolo',
        executable='origincar_yolo',
        name='origincar_yolo',
        output='screen',
        parameters=[{
            'sub_img_topic': '/aurora/rgb/image_raw',
            'config_file': yolo_config,
        }],
        arguments=['--ros-args', '--log-level', 'warn'],
    )

    # ── 5) YOLO→Cone 桥接 (/yolo_detections + depth → /cone_detected/distance/bearing) ─
    yolo_bridge = Node(
        package='origincar_yolo',
        executable='yolo_cone_bridge.py',
        name='yolo_cone_bridge',
        output='screen',
        parameters=[{
            'camera_fx': 416.49,
            'camera_fy': 417.383,
            'camera_cx': 316.832,
            'camera_cy': 188.753,
            'min_confidence': 0.25,
            'depth_topic': '/aurora/depth/image_raw',
            'yolo_topic': '/yolo_detections',
            'detection_timeout_s': 1.0,
        }],
    )

    # ── 6) 循迹 (cmd_vel 重映射到 /path_follower/cmd_vel) ────
    follower = Node(
        package='origincar_inertial_nav',
        executable='path_follower',
        name='path_follower',
        output='screen',
        parameters=[{
            'odom_topic': '/odom_combined',
            'cmd_vel_topic': '/path_follower/cmd_vel',
            'csv_path': LaunchConfiguration('resolved_csv', default=''),
            'rate': 20.0,
            'max_speed': LaunchConfiguration('max_speed', default='0.50'),
            'min_speed': LaunchConfiguration('min_speed', default='0.20'),
            'max_omega': LaunchConfiguration('max_omega', default='3.00'),
            'lat_acc_max': LaunchConfiguration('lat_acc_max', default='0.60'),
            'lookahead_min': 0.35,
            'lookahead_max': 1.0,
            'lookahead_k': LaunchConfiguration('lookahead_k', default='0.80'),
            'curvature_ff_gain': LaunchConfiguration('curvature_ff_gain', default='0.35'),
            'goal_tolerance': LaunchConfiguration('goal_tolerance', default='0.15'),
            'search_window': 60,
            'auto_start': False,
        }],
    )

    # ── 7) 状态机总控 (YOLO锥桶避障, /cmd_vel 唯一发布者) ────
    stage2 = Node(
        package='origincar_stage2',
        executable='stage2_node',
        name='stage2_state_machine',
        output='screen',
        parameters=[{
            'direction': direction,
            'cmd_vel_topic': '/cmd_vel',
            'follow_cmd_topic': '/path_follower/cmd_vel',
            'cone_topic': '/cone_detected',
            'cone_dist_topic': '/cone_distance',
            'cone_bearing_topic': '/cone_bearing',
            'finished_topic': '/path_follower/finished',
            'start_service': '/path_follower/start_following',
            'control_rate': 50.0,
            # ── 开环锥桶避障 (锥桶<触发距→往内环猛转+降速, 固定时长) ──
            'cone_avoid_enable': True,
            'cone_avoid_trigger_m': LaunchConfiguration('cone_avoid_trigger_m', default='0.55'),
            'cone_avoid_max_omega': LaunchConfiguration('cone_avoid_max_omega', default='5.00'),
            'cone_avoid_creep_v': LaunchConfiguration('cone_avoid_creep_v', default='0.40'),
            'cone_avoid_duration_s': LaunchConfiguration('cone_avoid_duration_s', default='0.60'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('direction', default_value='cw',
                              description='cw=顺时针, ccw=逆时针'),
        DeclareLaunchArgument('cw_csv', default_value=default_cw_csv),
        DeclareLaunchArgument('ccw_csv', default_value=default_ccw_csv),
        # 循迹
        DeclareLaunchArgument('max_speed', default_value='0.50'),
        DeclareLaunchArgument('min_speed', default_value='0.20'),
        DeclareLaunchArgument('max_omega', default_value='3.00'),
        DeclareLaunchArgument('lat_acc_max', default_value='0.60'),
        DeclareLaunchArgument('goal_tolerance', default_value='0.15'),
        DeclareLaunchArgument('lookahead_k', default_value='0.80',
                              description='前视距离增益, cw自动切0.50'),
        DeclareLaunchArgument('curvature_ff_gain', default_value='0.35',
                              description='曲率前馈增益, cw自动切0.20'),
        # 开环锥桶避障
        DeclareLaunchArgument('cone_avoid_trigger_m', default_value='0.55',
                              description='锥桶<此距(m)才触发避障'),
        DeclareLaunchArgument('cone_avoid_max_omega', default_value='5.00',
                              description='避让角速度(rad/s), bearing偏左→右转, bearing偏右→左转'),
        DeclareLaunchArgument('cone_avoid_creep_v', default_value='0.40',
                              description='避让前进速度(m/s)'),
        DeclareLaunchArgument('cone_avoid_duration_s', default_value='0.60',
                              description='避让持续时间(s)'),

        OpaqueFunction(function=_resolve_csv),
        OpaqueFunction(function=_resolve_direction_defaults),

        TimerAction(period=0.0, actions=[chassis_inertial, lslidar_node, aurora930, depth_camera_tf]),
        TimerAction(period=3.0, actions=[yolo_node, yolo_bridge]),
        TimerAction(period=5.0, actions=[follower]),
        TimerAction(period=7.0, actions=[stage2]),
    ])
