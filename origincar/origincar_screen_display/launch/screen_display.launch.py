#!/usr/bin/env python3
"""Start only the independent on-car result display node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('display_server', default_value=':0'),
        DeclareLaunchArgument('fullscreen', default_value='true'),
        DeclareLaunchArgument('hide_cursor', default_value='true'),
        DeclareLaunchArgument('qr_topic', default_value='/qr_result'),
        DeclareLaunchArgument('vlm_topic', default_value='/vlm/result'),
        Node(
            package='origincar_screen_display',
            executable='screen_display',
            name='result_screen_display',
            output='screen',
            parameters=[{
                'display_server': LaunchConfiguration('display_server'),
                'fullscreen': LaunchConfiguration('fullscreen'),
                'hide_cursor': LaunchConfiguration('hide_cursor'),
                'qr_topic': LaunchConfiguration('qr_topic'),
                'vlm_topic': LaunchConfiguration('vlm_topic'),
            }],
        ),
    ])
