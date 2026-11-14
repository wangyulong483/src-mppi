#!/usr/bin/env python3
"""
path_follower_node.py -- 实时循迹追踪 (段2 惯导功能包第二个节点)
================================================================
和 path_recorder 配套: recorder 录一条 CSV 参考轨迹, 本节点加载它作为参考,
用惯导 *实时闭环* 跟踪这条线 —— 每拍读当前 odom 位姿, 算与参考线的偏差,
Pure Pursuit(纯跟踪)算出前进速度+角速度, 发 /cmd_vel 纠偏跟线。
不是开环回放录制时的速度, 而是实时按当前位姿纠偏。

定位来源 (纯惯导, 与录制时完全一致):
  订阅 EKF 融合输出 /odom_combined (IMU + 编码器, odom 系), 不用雷达/地图。
  轨迹是相对开机点(odom 原点)的 —— 追踪时车必须从"和录制时相同的物理起点、
  odom 归零(重新开机)"开始, 否则整圈会整体偏移。

控制链 (差速模式, akmcar:=false):
  本节点发标准 Twist 到 /cmd_vel(linear.x=v, angular.z=ω), ω = v·κ。
  akmcar=false 时 origincar_base 直接把 (v, ω) 下发 STM32(差速内部解算),
  不经阿克曼转换链。纯跟踪几何: κ = 2·sin(alpha)/Ld。

跟踪律 (纯跟踪 + 抗漂移强化, 全程 xy 系几何结算):
  纯惯导绕半圈后车会"歪"—— 一部分是 odom 航向漂移(EKF 上游, 本节点管不到),
  另一部分是纯跟踪自身缺陷: 大前视会系统性切内弯, 每个弯往里切一点, 逐圈累积成偏移。
  这里在纯跟踪(前视纠偏)之上叠加两项, 把车"贴"在参考线上, 消掉本节点这部分歪斜:
    · 曲率前馈 κ_path : 用录制线自身几何曲率提前转弯, 不再等偏了才反应(消切弯)。
    · 横向/航向反馈   : 把车对参考线的带符号横向偏差 e、航向偏差 ψ 拉回零。
  ω = v·κ_pp + ff·v·κ_path − k_e·e − k_ψ·ψ, 限幅 max_omega。
  三个增益(curvature_ff_gain / cross_track_gain / heading_gain)全置 0 即退化回原纯跟踪,
  便于在车上逐项整定。κ_path、切向角均由参考线 xy 预算(轻度平滑抑噪)。

速度策略 (弯道减速自适应):
  直道给 max_speed; 曲率大时按侧向加速度上限 v = sqrt(a_lat / |κ|) 自动降速,
  下限 min_speed。既跑得稳、又不易冲出黄道。

终点减速 (防过冲, 关键安全):
  进入 goal_slowdown_dist 后, 线速度从巡航线性降到 goal_creep_v(可低于 min_speed)。
  段2直发 /cmd_vel 无 velocity_smoother 缓冲, 停车是一步归零; 若停车瞬间速度仍高,
  底盘惯性会冲过末点(实测曾冲 0.137m)。末点贴通道口/禁区边, 过冲即压线/撞障碍。
  用远低于 min_speed 的终点爬行速度收尾, 把停车瞬间动量压到最小, 从根上消除过冲。
  min_speed 仍作用于常规段(保证弯道转向权), 只有临近末点才放行到 goal_creep_v。

终点判定 (闭环安全):
  最近点索引"只前进不后退"(窗口搜索), 避免闭环轨迹里把起点附近误判成终点。
  跑到轨迹末点附近(< goal_tolerance)即停车。

控制服务 (std_srvs/Trigger):
  ~/start_following : 开始循迹 (auto_start=false 时靠它发车, 更安全)
  ~/stop_following  : 立即停车 (发零速)

参数:
  odom_topic       (string, '/odom_combined')
  cmd_vel_topic    (string, '/cmd_vel')
  csv_path         (string, '')  指定 CSV; 空则取 path_dir 里最新的 path_*.csv
  path_dir         (string, '~/origincar_paths')
  rate             (double, 20.0)   控制频率 (Hz)
  max_speed        (double, 0.30)   直道最大线速度 (m/s)
  min_speed        (double, 0.12)   最小线速度 (m/s, 保证阿克曼始终能转)
  lat_acc_max      (double, 0.40)   弯道侧向加速度上限 (m/s^2), 越小弯道越慢
  lookahead_min    (double, 0.25)   前视距离下限 (m)
  lookahead_max    (double, 0.60)   前视距离上限 (m)
  lookahead_k      (double, 0.6)    前视随速增益: Ld = clamp(min + k·v, min, max)
  lookahead_curv_ratio (double, 0.0) 曲率自适应前视: 进弯把 Ld 压到 ratio×本地转弯半径(1/κ),
                                    弯越急前视越短、越贴线, 直道(κ→0)不生效; 消切内弯; 0=关
  curvature_ff_gain (double, 0.5)   参考线曲率前馈增益: 提前按录制曲率转弯, 消切弯(0=关)
  cross_track_gain (double, 1.2)    横向偏差(CTE)拉回增益 (rad/s per m), 车偏离线时纠偏(0=关)
  heading_gain     (double, 0.8)    航向偏差拉回增益 (rad/s per rad), 车头与切向对齐(0=关)
  max_omega        (double, 1.5)    最大角速度 (rad/s), 差速转弯限幅
  goal_tolerance   (double, 0.15)   到末点多近算完成 (m)
  goal_slowdown_dist (double, 1.0)  距末点多远开始终点减速 (m)
  goal_creep_v     (double, 0.08)   终点爬行速度 (m/s, 可低于 min_speed), 压制停车过冲
  search_window    (int,    40)     最近点前向搜索窗口 (点数)
  auto_start       (bool,   False)  True=起来就跑(危险, 慎用); False=等 start 服务
"""

import os
import csv
import glob
import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import Trigger

from rclpy.qos import QoSProfile, QoSDurabilityPolicy

import tf2_ros
from rclpy.time import Time
from rclpy.duration import Duration
from rcl_interfaces.msg import ParameterDescriptor


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def wrap(a: float) -> float:
    """把角度归一化到 (-pi, pi]。"""
    return math.atan2(math.sin(a), math.cos(a))


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        # ── 参数 ─────────────────────────────────────────────
        self.odom_topic = self.declare_parameter('odom_topic', '/odom_combined').value
        self.cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').value
        self.csv_path = self.declare_parameter('csv_path', '').value
        self.path_dir = os.path.expanduser(
            self.declare_parameter('path_dir', '~/origincar_paths').value)
        self.rate = float(self.declare_parameter('rate', 20.0).value)
        self.max_speed = float(self.declare_parameter('max_speed', 0.50).value)
        self.min_speed = float(self.declare_parameter('min_speed', 0.12).value)
        self.lat_acc_max = float(self.declare_parameter('lat_acc_max', 0.60).value)
        self.lookahead_min = float(self.declare_parameter('lookahead_min', 0.25).value)
        self.lookahead_max = float(self.declare_parameter('lookahead_max', 0.60).value)
        self.lookahead_k = float(self.declare_parameter('lookahead_k', 0.6).value)
        # 曲率自适应前视 (消切内弯的结构性正解): 进弯把 Ld 压到"本地转弯半径 × ratio",
        # 本地半径由参考线曲率自算(1/κ) —— 弯越急前视越短越贴线, 直道 κ→0 时自动放大。
        # 无需知道赛道确切尺寸, 换赛道自适应。<=0 关闭 (退回纯速度前视, 保持旧行为)。
        self.lookahead_curv_ratio = float(
            self.declare_parameter('lookahead_curv_ratio', 0.0).value)
        # 抗漂移强化增益 (全置 0 即退化回原纯跟踪)
        self.curvature_ff_gain = float(self.declare_parameter('curvature_ff_gain', 0.35).value)
        self.cross_track_gain = float(self.declare_parameter('cross_track_gain', 1.2).value)
        self.heading_gain = float(self.declare_parameter('heading_gain', 0.8).value)
        self.max_omega = float(self.declare_parameter('max_omega', 3.00).value)
        self.goal_tolerance = float(self.declare_parameter('goal_tolerance', 0.15).value)
        # 终点减速: 进入 goal_slowdown_dist 后线性降到 goal_creep_v (可低于 min_speed),
        # 让停车瞬间速度足够低, 消除底盘惯性过冲(段2末点贴通道口/禁区边)。
        self.goal_slowdown_dist = float(self.declare_parameter('goal_slowdown_dist', 1.0).value)
        self.goal_creep_v = float(self.declare_parameter('goal_creep_v', 0.08).value)
        self.search_window = int(self.declare_parameter('search_window', 40).value)
        self.auto_start = bool(self.declare_parameter('auto_start', False).value)
        self.pub_waypoint_index = bool(self.declare_parameter('pub_waypoint_index', False).value)

        # ── VLM 拍照停车 (2026-07-22) ───────────────────────────
        # 到达指定路点索引时短暂停车，让 Aurora930 640x400 拍出清晰照片再继续。
        # 与 vlm_node 的 trigger_indices 共用同一套索引。
        # ★ 空列表默认值 → rclpy 推断为 BYTE_ARRAY, 必须 dynamic_typing=True
        #   才允许 launch 传入的 INTEGER_ARRAY 覆盖 (同 vlm_node._dyn 修复)。
        _dyn = ParameterDescriptor(dynamic_typing=True)
        self.vlm_pause_indices = self.declare_parameter(
            'vlm_pause_indices', [], descriptor=_dyn).value
        if isinstance(self.vlm_pause_indices, str):
            import json
            self.vlm_pause_indices = json.loads(self.vlm_pause_indices) if self.vlm_pause_indices else []
        self.vlm_pause_indices = set(int(x) for x in (self.vlm_pause_indices or []))
        self.vlm_pause_duration = float(self.declare_parameter('vlm_pause_duration', 1.5).value)
        # 距路点多近触发停车 (m)。odom 有漂移, 太小容易错过, 太大过早停车。
        self.vlm_pause_radius = float(self.declare_parameter('vlm_pause_radius', 0.25).value)

        # ── 起点位姿对齐 (点位校准 #1/#4) ────────────────────
        # 录制轨迹是"相对录制时开机点"的。若本次开机点/交接点与录制时不同,
        # 用 origin_* 把整条参考轨迹刚体变换到"录制原点在当前 odom 系里的实际位姿"。
        # 默认 (0,0,0)=恒等: 车在同一物理点重新开机、odom 归零时无需改。
        # 段1(nav2) 交接时, 可把交接落点位姿传进来对齐。
        self.origin_x = float(self.declare_parameter('origin_x', 0.0).value)
        self.origin_y = float(self.declare_parameter('origin_y', 0.0).value)
        self.origin_yaw = float(self.declare_parameter('origin_yaw', 0.0).value)

        # ── 锚定模式 (联合段1→段2交接的核心修复) ─────────────
        # 'odom' (默认, 标准安全): 发车时把参考轨迹刚体贴到"当前 odom 位姿"。
        #        单独测段2时车就摆在示教起点, 完美; 但联合跑时 Nav2 停靠位姿≠示教
        #        起点 → 整条环道被刚体旋转/平移 → 飘。
        # 'map'  (联合修复): 发车时查 map→odom (AMCL 提供), 把环道锚到"录制时的
        #        绝对 map 起点位姿"(来自 CSV 边车 .origin 或 map_start_* 参数),
        #        与 Nav2 停在哪无关 → 环道始终盖在真实黄环上。
        self.anchor_mode = str(self.declare_parameter('anchor_mode', 'odom').value)
        self.map_frame = str(self.declare_parameter('map_frame', 'map').value)
        self.odom_frame = str(self.declare_parameter('odom_frame', 'odom').value)
        # 录制时环道起点的 map 位姿 P_rec_map; 留 NaN 则尝试读 CSV 边车 .origin
        nan = float('nan')
        self.map_start_x = float(self.declare_parameter('map_start_x', nan).value)
        self.map_start_y = float(self.declare_parameter('map_start_y', nan).value)
        self.map_start_yaw = float(self.declare_parameter('map_start_yaw', nan).value)
        self.anchor_tf_timeout = float(self.declare_parameter('anchor_tf_timeout', 2.0).value)

        # tf2: map→odom 查询 (仅 anchor_mode='map' 用)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── 加载参考轨迹 ─────────────────────────────────────
        self._th = []                # 参考线各点切向角 (rad), 由 xy 预算
        self._kappa = []             # 参考线各点曲率 (1/m), 由 xy 预算, 供前馈
        self._loaded_csv = ''        # 实际加载的 CSV 路径 (供 .origin 边车定位)
        self._path = self._load_path()   # [(x, y, yaw, v), ...]
        if not self._path:
            self.get_logger().error("未能加载任何轨迹点, 节点将不发指令。检查 csv_path / path_dir。")

        # ── 运行状态 ─────────────────────────────────────────
        self._odom = None            # (x, y, yaw)
        self._nearest = 0            # 最近点索引 (只前进不后退)
        self._last_v = 0.0           # 上一拍下发的线速度 (算前视用)
        self._following = False
        self._finished = False
        self._warned_odom = False
        # VLM 拍照停车
        self._vlm_pausing = False           # 当前是否在停车等待
        self._vlm_pause_start = 0.0         # 停车开始时刻 (ROS time seconds)
        self._vlm_visited = set()           # 已触发的路点索引 (避免重复停车)

        # ── 通信 ─────────────────────────────────────────────
        self._cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        # 到点完成信号 (latched, 供段3 状态机判定循迹结束)
        latched = QoSProfile(depth=1,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._finished_pub = self.create_publisher(Bool, '~/finished', latched)
        self._finished_pub.publish(Bool(data=False))
        # VLM 触发: 发布当前路点索引 (供 vlm_node 订阅)
        if self.pub_waypoint_index:
            self._wp_pub = self.create_publisher(Int32, '~/waypoint_index', 10)
        else:
            self._wp_pub = None
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)
        # 动态切路径: stage2 收到 QR 方向后发布目标 CSV 到此 topic
        self.create_subscription(String, '~/switch_path', self._on_switch_path, 10)
        self.create_service(Trigger, '~/start_following', self._srv_start)
        self.create_service(Trigger, '~/stop_following', self._srv_stop)
        self.create_service(Trigger, '~/reload_csv', self._srv_reload_csv)

        self._timer = self.create_timer(1.0 / max(self.rate, 1.0), self._tick)

        self.get_logger().info(
            f"path_follower 就绪(差速实时循迹): {len(self._path)} 点, 源={self.odom_topic}, "
            f"max_speed={self.max_speed:.2f} min_speed={self.min_speed:.2f} "
            f"Ld={self.lookahead_min:.2f}~{self.lookahead_max:.2f} "
            f"max_omega={self.max_omega:.2f} "
            f"终点减速: {self.goal_slowdown_dist:.2f}m内→爬行{self.goal_creep_v:.2f}m/s")

        if self.auto_start and self._path:
            self.get_logger().warn("auto_start=true: 3s 后自动发车! 车已摆到起点了吗?")
            self._start_timer = self.create_timer(3.0, self._auto_start_once)

    def _auto_start_once(self):
        self._start_timer.cancel()
        self._begin()

    # ── 加载 CSV ─────────────────────────────────────────────
    def _load_path(self):
        path_file = self.csv_path
        if not path_file:
            files = sorted(glob.glob(os.path.join(self.path_dir, 'path_*.csv')))
            if not files:
                self.get_logger().error(f"{self.path_dir} 下没有 path_*.csv")
                return []
            path_file = files[-1]     # 时间戳命名, 取最新
        path_file = os.path.expanduser(path_file)
        pts = []
        try:
            with open(path_file, 'r', newline='') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    pts.append((float(row['x']), float(row['y']),
                                float(row['yaw']), float(row.get('v', 0.0) or 0.0)))
        except (OSError, KeyError, ValueError) as e:
            self.get_logger().error(f"读取 {path_file} 失败: {e}")
            return []
        self._raw_path = [(x, y, yaw, v) for x, y, yaw, v in pts]  # 保存原始轨迹, 供发车时对齐
        self._loaded_csv = path_file
        self._maybe_load_origin_sidecar(path_file)   # 读 .origin 补 map 起点位姿
        pts = self._apply_origin(pts)
        self._th, self._kappa = self._build_geometry(pts)   # 预算切向角/曲率
        self.get_logger().info(f"已加载轨迹 {path_file} ({len(pts)} 点)")
        return pts

    def _maybe_load_origin_sidecar(self, path_file):
        """若 map_start_* 未显式给定(NaN), 尝试读取 CSV 边车 <csv>.origin,
        取录制时环道起点的 map 位姿 (map_x,map_y,map_yaw)。供 anchor_mode='map' 用。"""
        if not (math.isnan(self.map_start_x) or math.isnan(self.map_start_y)
                or math.isnan(self.map_start_yaw)):
            return   # 已由参数显式指定, 不覆盖
        sidecar = path_file + '.origin'
        if not os.path.isfile(sidecar):
            return
        try:
            with open(sidecar, 'r', newline='') as f:
                row = next(csv.DictReader(f))
                self.map_start_x = float(row['map_x'])
                self.map_start_y = float(row['map_y'])
                self.map_start_yaw = float(row['map_yaw'])
            self.get_logger().info(
                f"已读取录制起点 map 位姿(边车 .origin): "
                f"({self.map_start_x:.3f}, {self.map_start_y:.3f}, {self.map_start_yaw:.3f}rad)")
        except (OSError, KeyError, ValueError, StopIteration) as e:
            self.get_logger().warn(f"读取 {sidecar} 失败, 忽略: {e}")

    def _apply_origin(self, pts):
        """把参考轨迹刚体变换到 (origin_x, origin_y, origin_yaw)。默认恒等。"""
        if self.origin_x == 0.0 and self.origin_y == 0.0 and self.origin_yaw == 0.0:
            return pts
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        out = []
        for x, y, yaw, v in pts:
            X = self.origin_x + c * x - s * y
            Y = self.origin_y + s * x + c * y
            out.append((X, Y, yaw + self.origin_yaw, v))
        self.get_logger().info(
            f"参考轨迹已按起点 ({self.origin_x:.3f}, {self.origin_y:.3f}, "
            f"{self.origin_yaw:.3f}rad) 做刚体变换")
        return out

    def _build_geometry(self, path):
        """从参考线 xy 预算每点切向角 th(rad) 与曲率 kappa(1/m)。
        供跟踪律做曲率前馈与横向/航向反馈; 曲率做轻度滑动平均抑噪, 避免前馈抖动。
        对参考轨迹的刚体变换只旋转切向、不改曲率, 每次 path 变更后重算即可。"""
        n = len(path)
        if n == 0:
            return [], []
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        th = [0.0] * n
        for i in range(n):
            j0, j1 = max(0, i - 1), min(n - 1, i + 1)
            dx, dy = xs[j1] - xs[j0], ys[j1] - ys[j0]
            if abs(dx) > 1e-9 or abs(dy) > 1e-9:
                th[i] = math.atan2(dy, dx)
            elif i > 0:
                th[i] = th[i - 1]        # 重合点沿用前一点切向
        kap = [0.0] * n
        for i in range(n):
            j0, j1 = max(0, i - 1), min(n - 1, i + 1)
            ds = math.hypot(xs[j1] - xs[j0], ys[j1] - ys[j0])
            kap[i] = wrap(th[j1] - th[j0]) / ds if ds > 1e-6 else 0.0
        if n >= 5:                       # 曲率轻度滑动平均(窗口5)抑噪
            sm = kap[:]
            for i in range(2, n - 2):
                sm[i] = (kap[i - 2] + kap[i - 1] + kap[i] + kap[i + 1] + kap[i + 2]) / 5.0
            kap = sm
        return th, kap

    # ── odom 回调 ────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._odom = (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))

    # ── 服务 ─────────────────────────────────────────────────
    def _srv_start(self, request, response):
        if not self._path:
            response.success = False
            response.message = "无轨迹, 无法开始"
            return response
        if self._following:
            response.success = False
            response.message = "已在循迹中"
            return response
        self._begin()
        response.success = True
        response.message = f"开始循迹, 共 {len(self._path)} 点"
        return response

    def _on_switch_path(self, msg: String):
        """收到 stage2 的切路径指令: 更新 csv_path 并立即重载轨迹。"""
        new_path = msg.data
        if new_path and new_path != self.csv_path:
            self.csv_path = new_path
            self.get_logger().info(f"[switch_path] 收到切路径: {new_path}")
            # 不在这里重载 — 留给 _begin() 或 _srv_reload_csv 统一处理,
            # 避免跟在 start_following 前面 race。

    def _srv_stop(self, request, response):
        self._stop("收到 stop 服务")
        response.success = True
        response.message = "已停车"
        return response

    def _srv_reload_csv(self, request, response):
        """重载 CSV 轨迹: 重读 csv_path 参数, 供 stage2 收到 QR 方向后动态切路径。"""
        # 重读参数 (可能已被 stage2 通过 param client 改了)
        csv_val = self.get_parameter('csv_path').value
        if csv_val:
            self.csv_path = csv_val
        self.get_logger().info(f"重载轨迹: csv_path={self.csv_path}")
        self._path = self._load_path()
        if not self._path:
            response.success = False
            response.message = f"重载失败: {self.csv_path} 无轨迹点"
            return response
        self._nearest = 0
        self._last_v = 0.0
        self._finished = False
        self._following = False
        self._finished_pub.publish(Bool(data=False))
        response.success = True
        response.message = f"已重载 {len(self._path)} 点"
        self.get_logger().info(f"重载完成: {self._loaded_csv} ({len(self._path)} 点)")
        return response

    def _begin(self):
        # ★ 发车前重载轨迹: csv_path 可能已被 stage2 通过 ~/switch_path 更新
        #   (QR 方向 -> 动态切 path_cw.csv <-> path_ccw.csv)
        self._path = self._load_path()
        if not self._path:
            self.get_logger().error("发车失败: 无轨迹点")
            return
        self._nearest = 0
        self._last_v = 0.0
        self._finished = False
        self._following = True
        self._vlm_pausing = False
        self._vlm_pause_start = 0.0
        self._vlm_visited.clear()

        # ── 发车锚定 ─────────────────────────────────────────
        # 'map' 模式(联合修复): 用 AMCL 的 map→odom 把环道锚到"录制时的绝对 map
        #   起点位姿", 与 Nav2 停在哪无关 → 环道盖在真实黄环上。失败则回退 'odom'。
        if self.anchor_mode == 'map' and self._begin_anchor_map():
            pass
        elif self._raw_path and self._odom is not None:
            # 'odom' 模式(默认/回退): 把轨迹刚体贴到"当前 odom 位姿"。
            # 单独测段2(车摆在示教起点)完美; 联合跑若 Nav2 停靠位姿≠示教起点会飘。
            ox, oy, oyaw = self._odom
            if (abs(ox - self.origin_x) > 1e-3 or abs(oy - self.origin_y) > 1e-3
                    or abs(oyaw - self.origin_yaw) > 1e-3):
                self.origin_x, self.origin_y, self.origin_yaw = ox, oy, oyaw
                self._path = self._apply_origin(self._raw_path)
                self._th, self._kappa = self._build_geometry(self._path)
                self.get_logger().info(
                    f"参考轨迹已对齐到当前odom ({ox:.3f}, {oy:.3f}, {oyaw:.3f}rad)")

        self.get_logger().info("▶ 开始循迹")

    def _begin_anchor_map(self):
        """map 绝对锚定: origin = (map→odom) ∘ P_rec_map, 把录制环道放到真实物理位置。
        需要 map_start_*(录制 map 起点)与 map→odom TF(AMCL 在线)。成功返回 True。"""
        if not self._raw_path:
            return False
        if (math.isnan(self.map_start_x) or math.isnan(self.map_start_y)
                or math.isnan(self.map_start_yaw)):
            self.get_logger().warn(
                "anchor_mode=map 但无录制起点 map 位姿(缺 .origin/map_start_*) → 回退 odom 锚定")
            return False
        try:
            # map 原点在 odom 系里的位姿 (T_odom_map): 用它把 map 中的点换到 odom
            tf = self._tf_buffer.lookup_transform(
                self.odom_frame, self.map_frame, Time(),
                timeout=Duration(seconds=self.anchor_tf_timeout))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException, tf2_ros.TransformException) as e:
            self.get_logger().warn(
                f"anchor_mode=map 但查 {self.map_frame}→{self.odom_frame} TF 失败 "
                f"(AMCL 在线?): {e} → 回退 odom 锚定")
            return False

        t = tf.transform.translation
        q = tf.transform.rotation
        tyaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        c, s = math.cos(tyaw), math.sin(tyaw)
        # 复合 T_odom_map ∘ P_rec_map → 录制起点在当前 odom 系里的位姿 = 锚定 origin
        self.origin_x = t.x + c * self.map_start_x - s * self.map_start_y
        self.origin_y = t.y + s * self.map_start_x + c * self.map_start_y
        self.origin_yaw = wrap(tyaw + self.map_start_yaw)
        self._path = self._apply_origin(self._raw_path)
        self._th, self._kappa = self._build_geometry(self._path)
        self.get_logger().info(
            f"[map锚定] 录制起点 map({self.map_start_x:.3f},{self.map_start_y:.3f},"
            f"{self.map_start_yaw:.3f}) × map→odom({t.x:.3f},{t.y:.3f},{tyaw:.3f}) "
            f"→ odom锚点({self.origin_x:.3f},{self.origin_y:.3f},{self.origin_yaw:.3f}rad)")
        return True

    def _stop(self, reason: str):
        if self._following:
            self.get_logger().info(f"■ 停车: {reason}")
        self._following = False
        self._publish_cmd(0.0, 0.0)

    # ── 主循环: 纯跟踪 ───────────────────────────────────────
    def _tick(self):
        if self._finished or not self._following:
            return
        if self._odom is None:
            if not self._warned_odom:
                self.get_logger().warn(f"暂无 {self.odom_topic} (底盘/EKF 在线?), 等待中...")
                self._warned_odom = True
            self._publish_cmd(0.0, 0.0)
            return
        self._warned_odom = False

        x, y, yaw = self._odom
        n = len(self._path)

        # 1) 最近点索引: 从当前索引向前窗口搜索, 只前进不后退(闭环安全)
        best_i, best_d = self._nearest, self._dist(x, y, self._nearest)
        end = min(self._nearest + self.search_window, n - 1)
        i = self._nearest
        while i <= end:
            d = self._dist(x, y, i)
            if d < best_d:
                best_d, best_i = d, i
            i += 1
        self._nearest = best_i

        # 发布当前路点索引 (供 VLM 节点等订阅方使用)
        if self._wp_pub is not None:
            self._wp_pub.publish(Int32(data=self._nearest))

        # 1b) VLM 拍照停车: 到达触发路点 → 短暂停车 → 拍照清晰 → 继续
        if self.vlm_pause_indices:
            if self._vlm_pausing:
                # 停车中: 持续发零速, 等拍照窗口结束
                now = self.get_clock().now().nanoseconds * 1e-9
                if now - self._vlm_pause_start >= self.vlm_pause_duration:
                    self._vlm_pausing = False
                    self.get_logger().info(
                        f"[VLM] 拍照完成 (路点 {self._nearest}), 继续循迹")
                else:
                    self._publish_cmd(0.0, 0.0)
                    self._last_v = 0.0
                    return
            elif self._nearest not in self._vlm_visited:
                # 检查是否抵达某个 VLM 触发路点的停车半径内
                triggered = None
                for idx in self.vlm_pause_indices:
                    if idx in self._vlm_visited:
                        continue
                    d = math.hypot(self._path[idx][0] - x, self._path[idx][1] - y)
                    if d < self.vlm_pause_radius:
                        triggered = idx
                        break
                if triggered is not None:
                    self._vlm_visited.add(triggered)
                    self._vlm_pausing = True
                    self._vlm_pause_start = self.get_clock().now().nanoseconds * 1e-9
                    self.get_logger().info(
                        f"[VLM] 到达触发路点 {triggered}, 停车 {self.vlm_pause_duration:.1f}s 拍照...")
                    self._publish_cmd(0.0, 0.0)
                    self._last_v = 0.0
                    return

        # 2) 前视目标点: 从最近点向前找第一个距离 >= Ld 的点
        Ld = self.lookahead_min + self.lookahead_k * self._last_v
        # 曲率自适应: 进弯按"本地转弯半径 × ratio"压前视, 直道不受影响 → 从根上消切内弯。
        # 本地半径 = 1/|κ| (参考线自算); 弯越急前视越短越贴线。ratio<=0 时退回纯速度前视。
        if self.lookahead_curv_ratio > 0.0 and self._kappa:
            k_here = abs(self._kappa[self._nearest])
            if k_here > 1e-3:
                Ld = min(Ld, self.lookahead_curv_ratio / k_here)   # 不超过 ratio×本地半径
        Ld = clamp(Ld, self.lookahead_min, self.lookahead_max)
        tgt = self._nearest
        while tgt < n - 1 and self._dist(x, y, tgt) < Ld:
            tgt += 1
        tx, ty = self._path[tgt][0], self._path[tgt][1]

        # 3) 终点判定: 已到末点区且离末点足够近
        dist_goal = math.hypot(self._path[-1][0] - x, self._path[-1][1] - y)
        if tgt >= n - 1 and dist_goal < self.goal_tolerance:
            self._finish(dist_goal)
            return

        # 4) 纯跟踪(前视纠偏): 目标点变换到车体系求 alpha, 得纯跟踪曲率 κ_pp
        dx, dy = tx - x, ty - y
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        ld_actual = math.hypot(dx, dy)
        alpha = math.atan2(local_y, local_x)
        kappa_pp = 2.0 * math.sin(alpha) / max(ld_actual, 1e-3)

        # 4b) 抗漂移强化(全程 xy 几何结算): 参考线曲率前馈 + 横向/航向误差反馈。
        #     纯跟踪对大前视会系统性切内弯, 逐圈累积成"半圈就歪"; 用录制线自身曲率
        #     提前转弯(前馈), 再把车对参考线的带符号横向偏差 e、航向偏差 ψ 拉回零。
        px, py = self._path[self._nearest][0], self._path[self._nearest][1]
        th_p = self._th[self._nearest] if self._th else yaw
        kappa_ff = self._kappa[self._nearest] if self._kappa else 0.0
        ex, ey = x - px, y - py
        cte = -math.sin(th_p) * ex + math.cos(th_p) * ey   # 带符号横向偏差 (+左偏)
        psi = wrap(yaw - th_p)                             # 航向偏差 (车头 vs 参考切向)

        # 5) 速度: 按"纯跟踪与前馈中更急的曲率"降速, 提前为弯道减速(减少冲弯歪斜)
        kappa_curve = max(abs(kappa_pp), abs(kappa_ff))
        v = self.max_speed
        if kappa_curve > 1e-3:
            v = min(v, math.sqrt(self.lat_acc_max / kappa_curve))

        # 5b) 终点减速: 进入 goal_slowdown_dist 后, 线速度从巡航线性降到 goal_creep_v。
        #     直发 /cmd_vel 无 velocity_smoother 缓冲, 停车是一步归零 —— 若此刻速度仍高,
        #     底盘惯性会冲过末点(旧逻辑floor=min_speed=0.20, 实测冲 0.137m 压线/进禁区)。
        #     用 goal_creep_v(远低于 min_speed) 收尾, 把停车瞬间动量压到最小。
        near_goal = dist_goal < self.goal_slowdown_dist
        if near_goal:
            frac = clamp(dist_goal / max(self.goal_slowdown_dist, 1e-3), 0.0, 1.0)
            v_goal = self.goal_creep_v + (self.max_speed - self.goal_creep_v) * frac
            v = min(v, v_goal)

        # 6) 速度下限: 常规段用 min_speed 保证阿克曼转向权; 终点段放行到 goal_creep_v 便于刹停
        v_lo = self.goal_creep_v if near_goal else self.min_speed
        v = clamp(v, v_lo, self.max_speed)

        # 7) 下发 (差速): ω = v·κ_pp + ff·v·κ_path − k_e·cte − k_ψ·ψ, 钳制最大角速度。
        #    v·κ_pp   纯跟踪前视纠偏(原逻辑)
        #    ff·v·κ_path 曲率前馈: 按录制线曲率提前转弯, 消系统性切内弯
        #    −k_e·cte / −k_ψ·ψ  横向/航向反馈: 把车拉回参考线并对齐切向
        omega = (v * kappa_pp
                 + self.curvature_ff_gain * v * kappa_ff
                 - self.cross_track_gain * cte
                 - self.heading_gain * psi)
        omega = clamp(omega, -self.max_omega, self.max_omega)
        self._publish_cmd(v, omega)
        self._last_v = v

    def _finish(self, dist_goal):
        self._finished = True
        self._following = False
        self._publish_cmd(0.0, 0.0)
        self._finished_pub.publish(Bool(data=True))
        self.get_logger().info(
            f"✔ 到达终点, 停车 (末点距 {dist_goal:.3f}m, 最近点索引 {self._nearest}/{len(self._path)-1})")

    def _dist(self, x, y, i):
        return math.hypot(self._path[i][0] - x, self._path[i][1] - y)

    def _publish_cmd(self, v, omega):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(omega)
        self._cmd_pub.publish(msg)

    def destroy_node(self):
        # 收尾一定发零速, 避免节点退出后车还在跑
        try:
            self._publish_cmd(0.0, 0.0)
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except RuntimeError:
            pass


if __name__ == '__main__':
    main()
