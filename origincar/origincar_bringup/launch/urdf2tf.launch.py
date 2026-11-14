#!/usr/bin/env python3
"""
URDF -> TF: 用 robot_state_publisher 读取 origincar_slam.urdf, 发布全部固定连杆 TF
(base_footprint -> base_link -> laser_link, base_link -> bmi088_link)。
URDF 全为固定关节, 无需 joint_state_publisher。
参考 project_help/urdf2tf.launch.py。
"""

import os

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    description_dir = get_package_share_directory('origincar_description')
    default_model = os.path.join(description_dir, 'urdf', 'origincar_slam.urdf')

    declare_model = launch.actions.DeclareLaunchArgument(
        'model', default_value=str(default_model),
        description='URDF 绝对路径 (含 laser_link)')

    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['cat ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str)

    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    return launch.LaunchDescription([
        declare_model,
        robot_state_publisher,
    ])
