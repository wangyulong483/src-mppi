import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():
    # ============================================================
    # 深度相机 (Aurora930) + Web 展示 + 视频录制
    # 基于 depth_websocket_display.launch.py 增加 video_recorder 节点
    #
    # 录制控制:
    #   开始: ros2 service call /video_recorder/record std_srvs/srv/SetBool "{data: true}"
    #   停止: ros2 service call /video_recorder/record std_srvs/srv/SetBool "{data: false}"
    # ============================================================

    # ===== 深度相机安装位姿 (base_link → depth_camera_link) =====
    depth_cam_x = LaunchConfiguration('depth_cam_x')
    depth_cam_y = LaunchConfiguration('depth_cam_y')
    depth_cam_z = LaunchConfiguration('depth_cam_z')
    depth_cam_roll = LaunchConfiguration('depth_cam_roll')
    depth_cam_pitch = LaunchConfiguration('depth_cam_pitch')
    depth_cam_yaw = LaunchConfiguration('depth_cam_yaw')

    # ===== JPEG 编码质量 =====
    jpg_quality = LaunchConfiguration('jpg_quality')

    # ===== 视频录制参数 =====
    record_output_dir = LaunchConfiguration('record_output_dir')
    record_fps = LaunchConfiguration('record_fps')
    record_fourcc = LaunchConfiguration('record_fourcc')

    # ===== Aurora930 深度相机驱动 =====
    aurora_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('deptrum-ros-driver-aurora930'),
                'launch', 'aurora930_launch.py')
        ),
        launch_arguments={
            'rgb_enable': 'true',
            'depth_enable': 'true',
            'ir_enable': 'true',
            'point_cloud_enable': 'true',
            'rgb_fps': '15',
            'ir_fps': '15',
            'heart_enable': 'false',
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
            'in_format': 'bgr8',
            'out_format': 'jpeg',
            'sub_topic': '/aurora/rgb/image_raw',
            'pub_topic': '/image',
            'jpg_quality': jpg_quality,
        }],
        arguments=['--ros-args', '--log-level', 'warn'],
    )

    # ===== websocket Web 展示 =====
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('websocket') + '/launch/websocket.launch.py'),
        launch_arguments={
            'websocket_image_topic': '/image',
            'websocket_image_type': 'mjpeg',
            'websocket_only_show_image': 'True',
        }.items()
    )

    # ===== 视频录制节点 (订阅原始 bgr8，保证画质) =====
    video_recorder_node = Node(
        package='origincar_bringup',
        executable='video_recorder.py',
        name='video_recorder',
        output='screen',
        parameters=[{
            'image_topic': '/aurora/rgb/image_raw',
            'output_dir': record_output_dir,
            'filename_prefix': 'aurora930',
            'fps': record_fps,
            'fourcc': record_fourcc,
            'file_extension': '.mp4',
        }],
        arguments=['--ros-args', '--log-level', 'info'],
    )

    return LaunchDescription([
        DeclareLaunchArgument('depth_cam_x', default_value='0.14'),
        DeclareLaunchArgument('depth_cam_y', default_value='0.0'),
        DeclareLaunchArgument('depth_cam_z', default_value='0.05',
                              description='深度相机高度(m)'),
        DeclareLaunchArgument('depth_cam_roll', default_value='0.0'),
        DeclareLaunchArgument('depth_cam_pitch', default_value='0.0'),
        DeclareLaunchArgument('depth_cam_yaw', default_value='0.0'),
        DeclareLaunchArgument('jpg_quality', default_value='60.0',
                              description='mjpeg 编码质量 0-100'),
        DeclareLaunchArgument('record_output_dir', default_value='/tmp/video_records',
                              description='视频文件输出目录'),
        DeclareLaunchArgument('record_fps', default_value='15.0',
                              description='录制帧率'),
        DeclareLaunchArgument('record_fourcc', default_value='mp4v',
                              description='OpenCV fourcc 编码器: mp4v / avc1 / XVID'),
        aurora_node,
        camera_tf,
        jpeg_codec_node,
        web_node,
        video_recorder_node,
    ])
