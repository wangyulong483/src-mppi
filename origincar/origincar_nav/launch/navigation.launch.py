#!/usr/bin/env python3
"""
Nav2 导航启动 (任务 1: 大厅内 P 点 → 任务发布点, 阿克曼 + RPP)。

自包含版本 —— 已剥离另一工程的竞赛/视觉节点 (task_manager / qr_decoder /
aurora930 / speed_profile / pose_pid_corrector 等) 与深度相机。避障纯靠
Lslidar N10 的 /scan。

启动链 (TimerAction 错峰, 防 TF 树未就绪):
  t=0s  底盘 + EKF + IMU + TF (akmcar:=true → 阿克曼指令链)
  t=2s  Lslidar N10 (/scan) + base_link→laser_link 静态 TF
  t=3s  Keepout 禁区服务器 (mask + filter_info + lifecycle_manager)
  t=5s  Nav2 栈 (AMCL 定位 + SmacPlannerHybrid + RegulatedPurePursuit + BT)
  t=8s  RViz2

阿克曼指令链: Nav2 /cmd_vel(Twist) → cmd_vel_to_ackermann_drive.py →
             /ackermann_cmd → origincar_base_node。

用法:
  ros2 launch origincar_nav navigation.launch.py map:=/path/to/hall_map.yaml
  # 默认 map 为 room.yaml; 实际请传入建好的大厅地图。
  # RViz 用 2D Pose Estimate 给 P 点初始位姿, 用 Nav2 Goal 指定任务发布点。

行为树路径注入:
  Humble navigation_launch.py 不覆写 BT XML 参数, 裸文件名需作为绝对路径写入
  YAML 才能被 bt_navigator 可靠加载。用 regex 替换 + 复制到 /tmp 作后备。
"""

import os
import re
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _patch_bt_path(yaml_path, old_filename, new_abs_path):
    """把 YAML 里裸 BT XML 文件名替换为绝对路径 (regex, 容错空格/引号)。返回临时 YAML 路径。"""
    with open(yaml_path, 'r') as f:
        content = f.read()
    escaped = re.escape(old_filename)
    for key in ('default_nav_to_pose_bt_xml', 'default_nav_through_poses_bt_xml'):
        pattern = r'(' + key + r'\s*:\s*)"?' + escaped + r'"?'
        replacement = r'\1"' + new_abs_path + '"'
        content = re.sub(pattern, replacement, content)
    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='_' + old_filename.replace('.xml', '.yaml'),
        delete=False, dir='/tmp')
    tmp.write(content)
    tmp.close()
    return tmp.name


def generate_launch_description():
    nav_dir = get_package_share_directory('origincar_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_dir = get_package_share_directory('origincar_bringup')

    rviz_config = os.path.join(nav_dir, 'rviz', 'nav.rviz')
    rpp_param_orig = os.path.join(nav_dir, 'param', 'originbot_nav2_rpp.yaml')

    bt_to_pose = os.path.join(nav_dir, 'behavior_trees', 'navigate_to_pose_ackermann_rpp.xml')
    bt_through_poses = os.path.join(nav_dir, 'behavior_trees', 'navigate_through_poses_ackermann_rpp.xml')

    # 复制 BT 到 /tmp 作后备路径
    shutil.copy(bt_to_pose, '/tmp/navigate_to_pose_ackermann_rpp.xml')
    shutil.copy(bt_through_poses, '/tmp/navigate_through_poses_ackermann_rpp.xml')

    # 生成写入绝对 BT 路径的临时 YAML
    rpp_param = _patch_bt_path(rpp_param_orig, 'navigate_to_pose_ackermann_rpp.xml', bt_to_pose)
    rpp_param = _patch_bt_path(rpp_param, 'navigate_through_poses_ackermann_rpp.xml', bt_through_poses)

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_path = LaunchConfiguration(
        'map', default=os.path.join(nav_dir, 'maps', 'room.yaml'))
    # ★ 空白 keepout mask (无禁区) → 导航立即可用。
    #   建好大厅地图后, 把大厅以外 (B/C 区) 对应的像素涂黑,
    #   就变成"仅大厅可通行"的禁区。不需要改任何代码。
    use_keepout = LaunchConfiguration('use_keepout', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation clock if true'),
        DeclareLaunchArgument('map',
                              default_value=os.path.join(nav_dir, 'maps', 'room.yaml'),
                              description='Full path to the map yaml (build a hall map first)'),
        DeclareLaunchArgument('use_keepout', default_value='true',
                              description='Enable keepout mask (默认 blank=无禁区, 涂黑 B/C 区即可)'),

        # t=0s: 底盘(差速) + Lslidar N10(/scan) + urdf2tf + odom2tf
        #       → Nav2 发 /cmd_vel, STM32 内部做阿克曼转换
        TimerAction(period=0.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'launch', 'bringup.launch.py')),
            ),
        ]),

        # t=3s: Keepout 禁区服务器 (mask + filter_info + lifecycle_manager)
        # 默认关闭 (use_keepout:=false) — 见上方说明。
        TimerAction(period=3.0, condition=IfCondition(use_keepout), actions=[
            Node(
                package='nav2_map_server', executable='map_server',
                name='keepout_mask_server',
                parameters=[{
                    'yaml_filename': os.path.join(nav_dir, 'maps', 'keepout_mask.yaml'),
                    'topic_name': '/keepout_mask',
                    'frame_id': 'map',
                }], output='screen'),
            Node(
                package='nav2_map_server', executable='costmap_filter_info_server',
                name='keepout_filter_info_server',
                parameters=[{
                    'type': 0,  # 0 = keepout
                    # ★ 必须与 param 里 keepout_filter.filter_info_topic 一致(=/costmap_ctrl)。
                    #   原工程由已删除的 competition_master 桥接, 这里直接发布到 /costmap_ctrl。
                    'filter_info_topic': '/costmap_ctrl',
                    'mask_topic': '/keepout_mask',
                    'base': 0.0, 'multiplier': 1.0,
                }], output='screen'),
            Node(
                package='nav2_lifecycle_manager', executable='lifecycle_manager',
                name='lifecycle_manager_keepout', output='screen',
                parameters=[{
                    'use_sim_time': False,
                    'autostart': True,
                    'node_names': ['keepout_mask_server', 'keepout_filter_info_server'],
                }]),
        ]),

        # t=5s: Nav2 栈 (SmacPlannerHybrid + RegulatedPurePursuit + 自定义 BT)
        TimerAction(period=5.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
                launch_arguments={
                    'map': map_yaml_path,
                    'use_sim_time': use_sim_time,
                    'params_file': rpp_param,
                    'use_composition': 'False',
                }.items(),
            ),
        ]),

        # t=8s: RViz2
        TimerAction(period=8.0, actions=[
            Node(
                package='rviz2', executable='rviz2', name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen',
            ),
        ]),
    ])
