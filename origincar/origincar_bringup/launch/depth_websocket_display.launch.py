import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():
    # ============================================================
    # 深度相机 (Aurora930) + Web 展示  —— 本工作空间唯一的视觉接口入口
    # 由原 USB 摄像头方案 (usb_websocket_display) 改造而来：
    #   旧: hobot_usb_cam(/dev/video8, mjpeg) → codec(mjpeg→nv12) → dnn → web
    #   新: Aurora930(/aurora/rgb/image_raw, bgr8) → codec(bgr8→jpeg) → web
    # 说明:
    #   - Aurora RGB 是 bgr8 原始图 (640x400)，不是 USB 的 mjpeg，
    #     所以 codec 从 "mjpeg→nv12" 改为 "bgr8→jpeg"(已在真机验证, ~14.7fps)。
    #   - 只做 Web 预览，不含 YOLO 推理 (dnn_node_example 已移除)；
    #     目标检测/二维码由 racing_vision_ai / qr_decoder 独立处理。
    #   - 本文件自带 Aurora930 驱动 + depth_camera_link 静态 TF，独立可跑。
    # ============================================================

    # ===== 深度相机安装位姿 (base_link → depth_camera_link) =====
    # 深度相机比原 USB 相机高约 5cm，z 默认 0.05m
    depth_cam_x = LaunchConfiguration('depth_cam_x')
    depth_cam_y = LaunchConfiguration('depth_cam_y')
    depth_cam_z = LaunchConfiguration('depth_cam_z')
    depth_cam_roll = LaunchConfiguration('depth_cam_roll')
    depth_cam_pitch = LaunchConfiguration('depth_cam_pitch')
    depth_cam_yaw = LaunchConfiguration('depth_cam_yaw')

    # ===== JPEG 编码质量 =====
    jpg_quality = LaunchConfiguration('jpg_quality')

    # ===== Aurora930 深度相机驱动 (取代 hobot_usb_cam) =====
    aurora_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('deptrum-ros-driver-aurora930'),
                'launch', 'aurora930_launch.py')
        ),
        launch_arguments={
            'rgb_enable': 'true',
            'depth_enable': 'true',
            'ir_enable': 'true',      # IR=深度传感器，必须开启才有 depth/points2
            'point_cloud_enable': 'true',
            'rgb_fps': '15',          # Aurora930 硬件上限 15fps
            'ir_fps': '15',
            'heart_enable': 'false',  # SDK v1.1.22 与固件 v1.7.2 心跳协议不兼容
            'stream_sdk_log_enable': 'false',
        }.items()
    )

    # ===== depth_camera_link 静态 TF =====
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='depth_camera_tf',
        arguments=[
            '--x', depth_cam_x,
            '--y', depth_cam_y,
            '--z', depth_cam_z,
            '--roll', depth_cam_roll,
            '--pitch', depth_cam_pitch,
            '--yaw', depth_cam_yaw,
            '--frame-id', 'base_link',
            '--child-frame-id', 'depth_camera_link',
        ],
        output='screen',
    )

    # ===== bgr8 → jpeg 编码 (websocket 需要 mjpeg) =====
    jpeg_codec_node = Node(
        package='hobot_codec',
        executable='hobot_codec_republish',
        name='hobot_codec_rgb2jpeg',
        output='screen',
        parameters=[{
            'channel': 1,
            'in_mode': 'ros',
            'out_mode': 'ros',
            'in_format': 'bgr8',            # ← Aurora RGB 编码
            'out_format': 'jpeg',
            'sub_topic': '/aurora/rgb/image_raw',
            'pub_topic': '/image',
            'jpg_quality': jpg_quality,
        }],
        arguments=['--ros-args', '--log-level', 'warn'],
    )

    # ===== websocket Web 展示 (http://<机器人IP>:8000, 仅显示图像) =====
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('websocket') + '/launch/websocket.launch.py'),
        launch_arguments={
            'websocket_image_topic': '/image',
            'websocket_image_type': 'mjpeg',
            'websocket_only_show_image': 'True',  # 无 AI 智能话题，仅显示图像
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('depth_cam_x', default_value='0.14'),
        DeclareLaunchArgument('depth_cam_y', default_value='0.0'),
        DeclareLaunchArgument('depth_cam_z', default_value='0.05',
                              description='深度相机高度(m)，比原USB高约5cm'),
        DeclareLaunchArgument('depth_cam_roll', default_value='0.0'),
        DeclareLaunchArgument('depth_cam_pitch', default_value='0.0'),
        DeclareLaunchArgument('depth_cam_yaw', default_value='0.0'),
        DeclareLaunchArgument('jpg_quality', default_value='60.0',
                              description='mjpeg 编码质量 0-100'),
        aurora_node,
        camera_tf,
        jpeg_codec_node,
        web_node,
    ])
