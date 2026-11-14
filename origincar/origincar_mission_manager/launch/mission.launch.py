#!/usr/bin/env python3
"""
mission.launch.py — 三赛段统一启动文件 (段1+3合并版)

启动链 (TimerAction 错峰, 保证依赖就绪):
  t=0s   底盘+惯导+TF   (origincar_bringup.launch.py, EKF 融合 IMU+轮速)
  t=0s   Lslidar N10    (/scan, frame_id=laser_link)
  t=0s   Aurora930相机   (深度+RGB+IR, 供QR扫码和锥桶检测)
  t=2s   Nav2 导航栈    (AMCL + SmacPlannerHybrid + RPP + BT)
  t=2s   锥桶探测        (fused_cone_detector, lidar_only PCA线性度)
  t=4s   QR解码+决策    (qr_decoder/WeChatQRCode + qr_decision, 发布 /qr_event)
  t=4s   段2循迹         (path_follower, 订阅 /odom_combined)
  t=6s   段2状态机       (stage2_state_machine, /cmd_vel 唯一发布者)
  t=6s   段1+3状态机     (mission_state_machine, Nav2出站+QR+归程统一执行)
  t=8s   任务调度者      (Mission Manager, 协调出站/段2/入站顺序+AMCL生命周期)

用法:
  ros2 launch origincar_mission_manager mission.launch.py direction:=cw map:=/path/to/hall_map.yaml

方向:
  direction:=cw   顺时针走环道
  direction:=ccw  逆时针走环道
"""

import os
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ── 方向相关参数 (联合版): (name, 原始默认, cw默认, ccw默认) ──
_DIRECTION_PARAMS = [
    ('curvature_ff_gain',       '0.35', '0.20', '0.35'),
    ('lookahead_k',             '0.40', '0.50', '0.40'),
    ('cone_avoid_creep_v',      '0.40', '0.40', '0.40'),
    ('cone_avoid_duration_s',   '0.60', '0.60', '0.60'),
    ('cone_avoid_max_omega',    '5.00', '5.00', '5.00'),
]


def _patch_bt_path(context):
    """把 Nav2 参数文件里的 BT XML 相对路径改为绝对路径, 写到临时文件。

    注意: Humble 的 bt_navigator 参数是 default_nav_to_pose_bt_xml /
    default_nav_through_poses_bt_xml (不是旧版 bt_xml_filename)。
    直接替换裸 XML 文件名，不依赖外层 YAML key 名，避免字符串匹配失败。
    """
    nav_share = get_package_share_directory('origincar_nav')
    yaml_path = os.path.join(nav_share, 'param', 'originbot_nav2_rpp.yaml')

    # 复制 BT XML 到 /tmp 作后备路径 (文件系统只读等极端情况兜底)
    for bt_name in ['navigate_to_pose_ackermann_rpp.xml',
                    'navigate_through_poses_ackermann_rpp.xml',
                    'navigate_to_pose_ackermann_mppi.xml']:
        src = os.path.join(nav_share, 'behavior_trees', bt_name)
        shutil.copy(src, os.path.join('/tmp', bt_name))

    with open(yaml_path, 'r') as f:
        config = f.read()
    for bt_name in ['navigate_to_pose_ackermann_rpp.xml',
                    'navigate_through_poses_ackermann_rpp.xml',
                    'navigate_to_pose_ackermann_mppi.xml']:
        abs_path = os.path.join(nav_share, 'behavior_trees', bt_name)
        # 替换带引号的相对路径 (Humble YAML 格式: key: "filename.xml")
        config = config.replace('"' + bt_name + '"', '"' + abs_path + '"')
        # 兜底: 替换不带引号的相对路径 (兼容其他写法)
        if abs_path not in config:
            config = config.replace(bt_name, abs_path)
    tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    tmp.write(config)
    tmp.close()
    context.launch_configurations['patched_nav2_params'] = tmp.name
    return []


def _resolve_csv(context):
    """按 direction 选顺/逆时针路径 CSV。"""
    direction = context.launch_configurations.get('direction', 'cw').lower()
    cw_csv = context.launch_configurations['cw_csv']
    ccw_csv = context.launch_configurations['ccw_csv']
    csv = ccw_csv if direction == 'ccw' else cw_csv
    csv = os.path.expanduser(csv)
    if not os.path.isfile(csv):
        print(f"[mission.launch] 警告: 找不到 {direction} 路径 CSV: {csv}")
    print(f"[mission.launch] direction={direction}, 路径: {csv}")
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
    # ── 共享目录 ──────────────────────────────────────────────
    base_share = get_package_share_directory('origincar_base')
    nav_share = get_package_share_directory('origincar_nav')
    inertial_share = get_package_share_directory('origincar_inertial_nav')
    comp_share = get_package_share_directory('origincar_compition')
    lslidar_share = get_package_share_directory('lslidar_driver')
    nav2_share = get_package_share_directory('nav2_bringup')
    aurora_share = get_package_share_directory('deptrum-ros-driver-aurora930')
    vlm_share = get_package_share_directory('origincar_vlm')

    lslidar_params = os.path.join(lslidar_share, 'params', 'lsx10.yaml')
    mission_params = os.path.join(comp_share, 'config', 'mission_params.yaml')
    map_yaml = LaunchConfiguration('map',
        default=os.path.join(nav_share, 'maps', 'room.yaml'))

    direction = LaunchConfiguration('direction', default='cw')

    default_cw_csv = os.path.join(inertial_share, 'config', 'path_cw.csv')
    default_ccw_csv = os.path.join(inertial_share, 'config', 'path_ccw.csv')

    # ── 1) t=0s: 底盘 + EKF + IMU (差速模式) ──────────────────
    chassis = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(base_share, 'launch', 'origincar_bringup.launch.py')),
        launch_arguments={'akmcar': 'false'}.items(),
    )

    # ── 2) t=0s: Lslidar N10 激光雷达 ─────────────────────────
    lslidar = Node(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        parameters=[lslidar_params],
    )

    # ── 3) t=0s: Aurora930 深度相机 (全开: 段1扫码用RGB, 段2锥桶用深度) ─
    aurora = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(aurora_share, 'launch', 'aurora930_launch.py')),
        launch_arguments={
            'rgb_enable': 'true',
            'depth_enable': 'true',
            'ir_enable': 'true',
            'point_cloud_enable': 'false',
            'rgb_fps': '15',
            'ir_fps': '15',
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

    # 预装填: Nav2 lifecycle 不自启, 等 ~/go 服务触发 (定义在前, keepout和nav2共用)
    autostart_nav2 = LaunchConfiguration('autostart', default='true')

    # ── 3.5) t=1s: Keepout 禁区掩膜服务器 (消弭 costmap keepout_filter 警告) ──
    use_keepout = LaunchConfiguration('use_keepout', default='true')
    skip_seg2 = LaunchConfiguration('skip_seg2', default='false')
    keepout_mask_server = Node(
        package='nav2_map_server', executable='map_server',
        name='keepout_mask_server',
        parameters=[{
            'yaml_filename': os.path.join(nav_share, 'maps', 'keepout_mask.yaml'),
            'topic_name': '/keepout_mask',
            'frame_id': 'map',
        }], output='screen')
    keepout_filter_info_server = Node(
        package='nav2_map_server', executable='costmap_filter_info_server',
        name='keepout_filter_info_server',
        parameters=[{
            'type': 0,  # 0 = keepout
            'filter_info_topic': '/costmap_ctrl',
            'mask_topic': '/keepout_mask',
            'base': 0.0, 'multiplier': 1.0,
        }], output='screen')
    keepout_lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_keepout', output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': autostart_nav2,
            'node_names': ['keepout_mask_server', 'keepout_filter_info_server'],
        }])

    # ── 4) t=2s: Nav2 导航栈 (预装填: autostart 参数已在上面定义) ──
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'params_file': LaunchConfiguration('patched_nav2_params'),
            'use_sim_time': 'false',
            'autostart': autostart_nav2,
        }.items(),
    )

    # ── 5) t=2s: YOLO BPU 锥桶检测 (RDK X5, Aurora930 RGB → /yolo_detections) ──
    stage2_share = get_package_share_directory('origincar_stage2')
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
    # YOLO → Cone 桥接: PerceptionTargets → /cone_detected + /cone_distance + /cone_bearing
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

    # ── 6) t=4s: QR 解码 + 方向决策 (lifecycle managed) ──────
    qr_decoder = Node(
        package='origincar_qr',
        executable='qr_decoder',
        name='qr_decoder_zbar',
        output='screen',
        parameters=[{
            'rgb_topic': '/aurora/rgb/image_raw',
            'depth_topic': '/aurora/depth/image_raw',
            'enable_depth_filter': True,
            'depth_min_mm': 50,
            'depth_max_mm': 4000,
            'model_dir': '/usr/share/wechat_qrcode',
        }],
    )
    qr_decision = Node(
        package='origincar_qr',
        executable='qr_decision',
        name='qr_decision',
        output='screen',
        parameters=[{'cooldown_sec': 3.0}],
    )
    # qr_decoder 已改为普通 Node + ~/enable (SetBool) 服务,
    # 由 mission_manager 在 SEG1_3_OUT 开始时 enable, 出站完成后 disable。
    # 不再需要 lifecycle_manager_qr (去 Lifecycle 化, 解决 Humble get_state 竞赛导致激活失败)。

    # ── 7) t=4s: 段2 循迹 (cmd_vel → /path_follower/cmd_vel, 不直发) ─
    path_follower = Node(
        package='origincar_inertial_nav',
        executable='path_follower',
        name='path_follower',
        output='screen',
        parameters=[{
            'odom_topic': '/odom_combined',
            'cmd_vel_topic': '/path_follower/cmd_vel',
            'csv_path': LaunchConfiguration('resolved_csv', default=''),
            'rate': 20.0,
            'max_speed': LaunchConfiguration('max_speed', default='0.45'),
            'min_speed': 0.12,
            'max_omega': 3.00,
            'lat_acc_max': 0.80,     # 弯道侧向加速度上限; 0.60→0.80, 弯中不减速太猛
            'lookahead_min': 0.25,
            'lookahead_max': 0.60,
            'lookahead_k': LaunchConfiguration('lookahead_k', default='0.40'),
            # 曲率前馈: 提前按录制曲率转弯, 消切内弯; 0=关(退化纯跟踪)
            'curvature_ff_gain': LaunchConfiguration('curvature_ff_gain', default='0.35'),
            # 曲率自适应前视: 弯道 Ld 压到 ≈本地转弯半径×0.15, 消"提前切内弯"
            'lookahead_curv_ratio': 0.15,
            # 横向/航向反馈 (仿真最优, 2026-07-22): 把小前视+强反馈配合, 贴线不振荡
            'cross_track_gain': 1.8,
            'heading_gain': 1.0,
            'goal_tolerance': 0.15,
            # ── 终点减速 (防惯性过冲, 2026-07-19) ──────────────────────────
            # 段2直发 /cmd_vel 无 velocity_smoother, 停车一步归零; min_speed=0.12
            # 弯道已可自然降速, 终点段 1.0m 内线性降到 0.08m/s 爬行收尾。
            'goal_slowdown_dist': 1.0,
            'goal_creep_v': 0.08,
            'search_window': 60,
            'auto_start': False,           # 由 Mission Manager 调 start 服务发车
            'pub_waypoint_index': True,    # 发布路点索引, 供 VLM 节点触发
            # ── 联合交接绝对锚定 (2026-07-19, 修"联合飘、单独好") ──────────
            # 联合跑时 Nav2 停靠位姿≠示教起点, odom 锚定会把整条环道刚体旋转→飘。
            # map 锚定: 发车时用 AMCL 的 map→odom 把环道锚到"绝对 map 起点位姿",
            # 与 Nav2 停在哪无关 → 环道盖在真实黄环上。
            # ★ map 起点 = 黄色通道入口路点 = 惯导(seg2)起点, 故直接写死为入口路点值
            #   (与 compition mission_params.yaml 的 yellow 一致: 已含 +5cm 微偏)。
            #   无需重录 .origin 边车。若改了入口路点, 这三个值要同步改。
            'anchor_mode': 'map',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'map_start_x': 0.843,
            'map_start_y': -1.078,
            'map_start_yaw': 0.262,
        }],
    )

    # ── 8) t=6s: 段2 状态机 (YOLO锥桶避障, /cmd_vel 唯一发布者) ────
    stage2 = Node(
        package='origincar_stage2',
        executable='stage2_node',
        name='stage2_state_machine',
        output='screen',
        parameters=[{
            'direction': direction,
            'path_cw': LaunchConfiguration('cw_csv'),
            'path_ccw': LaunchConfiguration('ccw_csv'),
            'cmd_vel_topic': '/cmd_vel',
            'follow_cmd_topic': '/path_follower/cmd_vel',
            'cone_topic': '/cone_detected',
            'cone_dist_topic': '/cone_distance',
            'cone_bearing_topic': '/cone_bearing',
            'finished_topic': '/path_follower/finished',
            'start_service': '/path_follower/start_following',
            'auto_enable': False,
            'control_rate': 50.0,
            # ── 开环锥桶避障 (锥桶<触发距→往内环猛转+降速, 固定时长, 2026-07-22) ──
            'cone_avoid_enable': True,
            'cone_avoid_trigger_m': 0.5,        # 触发距离 (m)
            'cone_avoid_max_omega': LaunchConfiguration('cone_avoid_max_omega', default='5.00'),
            'cone_avoid_creep_v': LaunchConfiguration('cone_avoid_creep_v', default='0.40'),
            'cone_avoid_duration_s': LaunchConfiguration('cone_avoid_duration_s', default='0.60'),
        }],
    )

    # ── 9) t=6s: 段1+3 状态机 (Nav2出站+QR+归程统一执行, 由 Mission Manager 调起) ─
    seg1_mission = Node(
        package='origincar_compition',
        executable='mission_state_machine',
        name='mission_state_machine',
        output='screen',
        parameters=[mission_params,
                    {'skip_seg2': skip_seg2}],
    )

    # ── 9.5b) t=0s: 车载屏幕显示 (QR扫码结果 + VLM图生文, 全屏) ─
    screen_display = Node(
        package='origincar_screen_display',
        executable='screen_display',
        name='result_screen_display',
        output='screen',
        parameters=[{
            'display_server': ':0',
            'fullscreen': True,
            'hide_cursor': True,
            'qr_topic': '/qr_result',
            'vlm_topic': '/vlm/result',
        }],
    )

    # ── 9.6) t=6s: VLM 图生文 (段2 环道人形立牌识别, 云端 API) ──
    vlm_node = Node(
        package='origincar_vlm',
        executable='vlm_node',
        name='vlm_node',
        output='screen',
        parameters=[{
            'api_key': 'sk-ws-H.EHMYXEL.FrRF.MEQCIAwLkXMEuoaBtVf4BbPybwdZqeepKN3lmJ0svdOqa-PXAiBiDpwI-dEc81oAs_CyKc2phRc8yTJJNllB2Cq5MFmLzg',
            # cw/ccw 两套触发索引都传给 vlm_node; 由它订 QR 方向信号
            # (/sign_switch_sign_data) 在运行时切换, 与 stage2 切路径同步。
            'trigger_indices_cw': LaunchConfiguration('vlm_indices_cw'),
            'trigger_indices_ccw': LaunchConfiguration('vlm_indices_ccw'),
            'cooldown_sec': 5.0,
            'enable_debug': True,
            'debug_dir': '/userdata/dev_ws/vlm_debug',
        }],
        # VLM 节点自带 ~/enable 门控, 由 Mission Manager 在段2放行
    )

    # ── 10) t=8s: 任务调度者 (最后启动, 等所有子接口就绪) ────
    manager = Node(
        package='origincar_mission_manager',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        parameters=[{
            'prearm': LaunchConfiguration('prearm', default='false'),
            'skip_seg2': skip_seg2,
            'transition_duration': 0.5,
            'amcl_settle_sec': 2.0,
            'control_rate': 10.0,
            'cone_enable_service': '/yolo_cone_bridge/enable',
            'qr_enable_service': '/qr_decoder_zbar/enable',
        }],
    )

    # ── 0) t=0s: xset 关屏保/DPMS, 防止车载屏幕自动熄屏 ────────
    xset_off = ExecuteProcess(
        cmd=[['xset', 's', 'off'],
             ['xset', '-dpms'],
             ['xset', 's', 'noblank']],
        shell=False,
        name='xset_disable_screensaver',
    )

    return LaunchDescription([
        # 路径
        DeclareLaunchArgument('direction', default_value='cw',
            description='cw=顺时针, ccw=逆时针'),
        DeclareLaunchArgument('cw_csv', default_value=default_cw_csv),
        DeclareLaunchArgument('ccw_csv', default_value=default_ccw_csv),
        # 地图
        DeclareLaunchArgument('map',
            default_value=os.path.join(nav_share, 'maps', 'room.yaml')),
        DeclareLaunchArgument('use_keepout', default_value='true',
            description='启用 keepout 禁区掩膜 (消弭 costmap 警告)'),
        # 锥桶
        DeclareLaunchArgument('cone_threshold_m', default_value='1.00'),
        # 循迹
        # 段2循迹: max_speed=0.45 + Ld=0.20~0.50 + CTE_gain=1.8 + Hdg_gain=1.0
        # (2026-07-22仿真最优, RMS CTE 0.030→0.009m, 改善69%)
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Nav2 lifecycle autostart; false=预装填等~/go'),
        DeclareLaunchArgument('prearm', default_value='false',
                              description='mission_manager预装填: 等~/go再激活Nav2'),
        DeclareLaunchArgument('skip_seg2', default_value='false',
                              description='保底模式: 跳过段2环道, 只跑段1(出站)+段3(归程)'),
        DeclareLaunchArgument('max_speed', default_value='0.45'),
        DeclareLaunchArgument('curvature_ff_gain', default_value='0.35',
            description='曲率前馈增益, cw自动切0.20'),
        DeclareLaunchArgument('lookahead_k', default_value='0.40',
            description='前视距离增益, cw自动切0.50'),
        # 开环锥桶避障 (CW/CCW 统一)
        DeclareLaunchArgument('cone_avoid_max_omega', default_value='5.00',
            description='避让角速度(rad/s)'),
        DeclareLaunchArgument('cone_avoid_creep_v', default_value='0.40',
            description='避让前进速度(m/s)'),
        DeclareLaunchArgument('cone_avoid_duration_s', default_value='0.60',
            description='避让持续时间(s)'),

        # VLM 图生文: 触发路点索引 (JSON数组, 如 "[30, 31, 60, 61]")
        # 每个立牌配两个相邻索引, 避免 odom 漂移导致错过单个触发点
        # VLM 触发路点索引: 两套都传给 vlm_node, 由它订 QR 方向信号在运行时切换
        # (段2方向由二维码定, 不是 launch 参数; 见 vlm_node._on_direction)。
        # 立牌物理位置同一处, 但 cw/ccw 路点编号方向相反 → 索引区间不同 (2026/07/21 定标)。
        #   cw : 顶边右侧 ≈ 110~115
        #   ccw: 顶边右侧 ≈ 110~120 (逆时针从右往左穿顶边)
        # 需临时改: 传 vlm_indices_cw:="[...]" 或 vlm_indices_ccw:="[...]"。
        DeclareLaunchArgument('vlm_indices_cw',
            default_value='[110, 111, 112, 113, 114, 115]',
            description='顺时针 VLM 触发路点索引 (JSON 数组)'),
        DeclareLaunchArgument('vlm_indices_ccw',
            default_value='[110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120]',
            description='逆时针 VLM 触发路点索引 (JSON 数组)'),

        OpaqueFunction(function=_resolve_csv),
        OpaqueFunction(function=_resolve_direction_defaults),
        OpaqueFunction(function=_patch_bt_path),

        # ── 错峰启动链 (压缩延迟: 预热服务已缓存 .so, 可用更短间隔) ──
        # t=0s: 硬件驱动 + 屏幕
        TimerAction(period=0.0, actions=[xset_off, chassis, lslidar, aurora, depth_camera_tf,
                                        screen_display]),
        # t=0.5s: 禁区掩膜 (条件: use_keepout)
        TimerAction(period=0.5, condition=IfCondition(use_keepout),
                    actions=[keepout_mask_server, keepout_filter_info_server,
                             keepout_lifecycle_manager]),
        # t=1.0s: Nav2 导航栈 (段1+3共用, 始终启动)
        TimerAction(period=1.0, actions=[nav2]),
        # t=1.0s: 段2 YOLO锥桶 (skip_seg2=true 时跳过)
        TimerAction(period=1.0, condition=UnlessCondition(skip_seg2),
                    actions=[yolo_node, yolo_bridge]),
        # t=2.0s: QR + 段2循迹 (段2部分 skip_seg2=true 时跳过)
        TimerAction(period=2.0, actions=[qr_decoder, qr_decision]),
        TimerAction(period=2.0, condition=UnlessCondition(skip_seg2),
                    actions=[path_follower]),
        # t=3.0s: 段1+3状态机 (始终) + 段2状态机/VLM (skip_seg2=true 时跳过)
        TimerAction(period=3.0, actions=[seg1_mission]),
        TimerAction(period=3.0, condition=UnlessCondition(skip_seg2),
                    actions=[stage2, vlm_node]),
        # t=4.0s: 任务调度者 (最后)
        TimerAction(period=4.0, actions=[manager]),
    ])
