import os
from pathlib import Path
import launch
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace
import launch_ros.actions
from launch.conditions import UnlessCondition

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('origincar_base')
    launch_dir = os.path.join(bringup_dir, 'launch')
    ekf_config = Path(get_package_share_directory('origincar_base'), 'config', 'ekf.yaml')
    imu_config = Path(get_package_share_directory('origincar_base'), 'config', 'imu.yaml')

    
    carto_slam = LaunchConfiguration('carto_slam', default='false')
    carto_slam_dec = DeclareLaunchArgument('carto_slam',default_value='false')
            
    # akmcar=true 时走阿克曼指令链: base 订阅 ackermann_cmd, 并启动
    # cmd_vel_to_ackermann_drive.py 把上层 /cmd_vel(Twist) 转成 /ackermann_cmd。
    # 导航时需要 true; 建图/遥控默认 false(直接 Twist cmd_vel)。
    akmcar = LaunchConfiguration('akmcar', default='false')
    akmcar_dec = DeclareLaunchArgument('akmcar', default_value='false',
            description='true: Ackermann cmd chain (cmd_vel->ackermann_cmd); false: direct Twist cmd_vel')

    origincar_base = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'base_serial.launch.py')),
            launch_arguments={'akmcar': akmcar}.items(),
    )

    # ===== 全部固定连杆 TF 由 robot_state_publisher 读 origincar_slam.urdf 发布 =====
    # 与已验证可行的 origincar_bringup/bringup.launch.py 保持同一套 TF 布局:
    #   base_footprint→base_link (0 0 0.07), base_link→laser_link (0 0 0.075),
    #   base_link→bmi088_link (0 0 0, 待标定安装朝向, 改 URDF 即可)。
    # odom→base_footprint 由下面的 EKF 发布。
    # 弃用原来的 static_transform_publisher(base_footprint→base_link=0.41 0.12 0 是错的,
    # 真值应为 0 0 0.07), 也不再加载 origincar.urdf(根是 base_link, 无 base_footprint/laser_link)。
    slam_urdf = os.path.join(
            get_package_share_directory('origincar_description'), 'urdf', 'origincar_slam.urdf')
    robot_state_publisher_node = launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            arguments=[slam_urdf],
    )

    # ===== BMI088 IIO → /imu/data_raw 桥接 (取代下位机 MPU6050) =====
    # 桥接节点默认发布 /imu_data, 这里 remap 成 /imu/data_raw 供 madgwick 使用。
    bmi088_bridge = launch_ros.actions.Node(
            package='origincar_base',
            executable='bmi088_iio_bridge.py',
            name='bmi088_iio_bridge',
            output='screen',
            parameters=[{
                'rate': 100.0,
                'frame_id': 'bmi088_link',
                # 本机 IIO 无 scale 文件(已实测), 必须手动指定, 否则按 1.0 输出原始 LSB:
                #   gyro : ±1000°/s → 1000*pi/180/32768 ≈ 0.0005326 rad/s per LSB
                #          (弧线标定 imu/odom≈1.84≈2×, 确认量程是 ±1000 而非 ±2000)
                #   accel: 经验标定 9.80665 / 2884(静止|a|的LSB) ≈ 0.0034 m/s^2 per LSB
                'gyro_scale_override': 0.0005326,
                'accel_scale_override': 0.0034,
            }],
            remappings=[('/imu_data', '/imu/data_raw')],
    )

    imu_filter_node =  launch_ros.actions.Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[imu_config]
    )
    
    robot_ekf = launch_ros.actions.Node(
            condition=UnlessCondition(carto_slam),
            package='robot_localization', 
            executable='ekf_node', 
            parameters=[ekf_config],
            remappings=[("odometry/filtered", "odom_combined")]
            )
                              
    ld = LaunchDescription()

    ld.add_action(carto_slam_dec)
    ld.add_action(akmcar_dec)
    ld.add_action(origincar_base)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(bmi088_bridge)
    ld.add_action(imu_filter_node)
    ld.add_action(robot_ekf)

    return ld

