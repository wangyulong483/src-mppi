#!/usr/bin/env python3
"""
step_tune.py — 阶跃响应辨识：自动标定差速小车转向动力学
============================================================
在空地上跑一组阶跃转向测试，识别一阶+死区模型，输出最优 path_follower 增益。

用法:
  # 1. 先把车摆到空地上，确保周围 ~2m 无障碍
  # 2. 停掉所有发 /cmd_vel 的节点 (stage2, Nav2, path_follower…)
  # 3. 运行:
  python3 step_tune.py --ros-args -p test_duration:=3.0 -p omega_step:=2.0

输出:
  识别到的模型: K(增益), tau(时间常数秒), theta(死区秒)
  推荐参数: lookahead_min, cross_track_gain, heading_gain, max_omega

原理:
  差速小车转向动力学近似一阶+死区:
    G(s) = K * exp(-theta*s) / (tau*s + 1)
  从阶跃响应中拟合 K, tau, theta → 反推控制增益。

安全:
  - 测试前发 2 秒零速确保底盘就绪
  - 每次阶跃后恢复零速
  - Ctrl-C 立刻停车
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class StepTuner(Node):
    def __init__(self):
        super().__init__('step_tune')

        # ── 测试参数 ──────────────────────────────────────────
        self.linear_v = float(self.declare_parameter('test_linear_v', 0.20).value)
        # 自动算 omega_step: R ≈ 0.5m 转弯半径, 不饱和
        _auto_omega = round(self.linear_v / 0.5, 2)
        self.omega_step = float(self.declare_parameter(
            'omega_step', _auto_omega).value)
        self.step_duration = float(self.declare_parameter('step_duration', 0.8).value)
        self.test_count = int(self.declare_parameter('test_count', 4).value)

        # ── 数据 ──────────────────────────────────────────────
        self._yaw_rate = 0.0
        self._odom_time = 0.0
        self._records = []  # [(t, omega_cmd, yaw_rate), ...]

        # ── 通信 ──────────────────────────────────────────────
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._odom_sub = self.create_subscription(
            Odometry, '/odom_combined', self._on_odom, 10)

    def _on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)

        # 用 twist 直接读角速度 (更准, 不需要差分)
        self._yaw_rate = msg.twist.twist.angular.z
        self._odom_time = (msg.header.stamp.sec +
                           msg.header.stamp.nanosec * 1e-9)

    def _send_cmd(self, v, omega):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(omega)
        self._cmd_pub.publish(msg)

    def _get_yaw_rate(self) -> float:
        """等待一帧 odom, 返回当前 yaw rate"""
        rclpy.spin_once(self, timeout_sec=0.05)
        return self._yaw_rate

    def _baseline(self, duration=0.5):
        """测静止时的 yaw rate 噪声水平"""
        self._send_cmd(0.0, 0.0)
        time.sleep(0.3)
        samples = []
        t0 = time.time()
        while time.time() - t0 < duration:
            rclpy.spin_once(self, timeout_sec=0.01)
            samples.append(abs(self._yaw_rate))
        return max(samples) if samples else 0.01

    def run(self):
        self.get_logger().info('=' * 60)
        self.get_logger().info('  阶跃响应辨识 — 差速转向动力学标定')
        self.get_logger().info(f'  omega_step={self.omega_step} rad/s  '
                               f'duration={self.step_duration}s  '
                               f'tests={self.test_count}')
        self.get_logger().info('=' * 60)

        # ── 安全检查 ──────────────────────────────────────────
        noise = self._baseline(1.0)
        self.get_logger().info(f'静止 yaw rate 噪声: {noise:.4f} rad/s')

        # ── 阶跃测试 ──────────────────────────────────────────
        all_steps = []
        for i in range(self.test_count):
            self.get_logger().info(f'\n--- 测试 {i+1}/{self.test_count} ---')

            # 静止等 1s
            self._send_cmd(0.0, 0.0)
            time.sleep(1.0)

            # 给前进速度 + 阶跃转向
            sign = 1.0 if i % 2 == 0 else -1.0  # 交替左右转
            omega = sign * self.omega_step
            self._send_cmd(self.linear_v, omega)

            # 记录响应
            t0 = time.time()
            record = []
            while time.time() - t0 < self.step_duration + 1.5:
                rclpy.spin_once(self, timeout_sec=0.005)
                t = time.time() - t0
                record.append((t, omega, self._yaw_rate))
            all_steps.append(record)

            # 恢复零速
            self._send_cmd(0.0, 0.0)
            time.sleep(1.5)

            # 打印即时观测
            steady = [abs(r[2]) for r in record
                      if r[0] > 0.3 and r[0] < self.step_duration]
            if steady:
                avg = sum(steady) / len(steady)
                self.get_logger().info(f'  稳态 yaw_rate ≈ {avg:.3f} rad/s  '
                                       f'(cmd={omega:.1f}, 增益≈{avg/abs(omega):.2f})')

        # ── 系统辨识 ──────────────────────────────────────────
        K, tau, theta = self._identify(all_steps, noise)

        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('  辨识结果')
        self.get_logger().info(f'  转向增益  K     = {K:.3f}  (output rad/s / input rad/s)')
        self.get_logger().info(f'  时间常数  tau   = {tau:.3f} s')
        self.get_logger().info(f'  等效死区  theta = {theta:.3f} s')
        self.get_logger().info('=' * 60)

        # ── 增益计算 ──────────────────────────────────────────
        self._compute_gains(K, tau, theta)

    def _identify(self, all_steps, noise):
        """从多次阶跃中拟合一阶+死区模型 G(s)=K*exp(-theta*s)/(tau*s+1).
        对每次测试独立分析，取中位数，消除单次异常。"""
        Ks, taus, thetas = [], [], []

        for record in all_steps:
            # 只取阶跃阶段 (cmd != 0)
            step_data = [(t, cmd, yr) for t, cmd, yr in record if cmd != 0]
            if not step_data:
                continue

            # 稳态: 阶跃后 0.3s~结束 的平均 yaw_rate
            avg_cmd = abs(step_data[0][1])
            steady = [abs(yr) for t, _, yr in step_data
                      if t > 0.3 and t < self.step_duration]
            if not steady or avg_cmd == 0:
                continue
            avg_yaw = sum(steady) / len(steady)
            Ks.append(avg_yaw / avg_cmd)

            # 上升时间 (10%→90%)
            t10 = t90 = None
            for t, _, yr in step_data:
                val = abs(yr)
                if t10 is None and val > 0.10 * avg_yaw:
                    t10 = t
                if t90 is None and val > 0.90 * avg_yaw:
                    t90 = t
                    break
            if t10 and t90 and t90 > t10:
                taus.append((t90 - t10) / 2.2)

            # 死区: 第一个超过 3×噪声 的时间
            threshold = max(noise * 3, 0.05)
            for t, _, yr in step_data:
                if abs(yr) > threshold:
                    thetas.append(min(t, 0.2))
                    break

        if not Ks:
            self.get_logger().warn('无有效数据, 用默认值')
            return 0.85, 0.12, 0.04

        # 取中位数 (抗异常值)
        def median(lst):
            s = sorted(lst)
            return s[len(s) // 2]

        K = median(Ks or [0.85])
        tau = median(taus or [0.12])
        theta = median(thetas or [0.04])

        tau = max(tau, 0.05)
        theta = max(theta, 0.02)
        return K, tau, theta

    def _compute_gains(self, K, tau, theta):
        """根据辨识模型计算纯跟踪最优增益."""
        v = 0.45  # 巡航速度

        # 1) 前视距离: 至少反应时间×速度, 确保"看"到转向生效
        reaction_t = tau + theta  # 命令→生效的总延迟
        lookahead_raw = v * reaction_t
        lookahead_min = max(0.15, min(0.35, round(lookahead_raw, 2)))
        lookahead_max = round(max(lookahead_min + 0.15, lookahead_min * 2.0), 2)

        # 2) 曲率前馈: K 低说明转向响应慢, 需要更强前馈提前转弯
        #    ff ≈ 0.35 是经验基线, K 偏离 1.0 时按比例调
        ff = round(0.35 / max(K, 0.08), 2)
        curvature_ff_gain = max(0.20, min(0.80, ff))

        # 3) 横向反馈: K 低时反馈不宜过强 (odom 噪声会被放大)
        #    用 tau 做阻尼标度: 慢响应 → 小增益, 快响应 → 大增益
        ct = round(1.2 / max(tau, 0.05), 1)
        cross_track_gain = max(0.8, min(2.2, ct))

        # 4) 航向反馈: 阻尼比 0.5~0.8, 标度到 tau
        hd = round(0.6 * cross_track_gain * math.sqrt(max(tau, 0.05)), 1)
        heading_gain = max(0.4, min(1.5, hd))

        # 5) 最大角速度: 取测试稳态的 2x (有速度时转向更快)
        max_omega = round(max(K * self.omega_step * 2.5, 2.0), 1)
        max_omega = max(2.0, min(4.0, max_omega))

        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info(f'  推荐 path_follower 参数 (@ v={v:.2f} m/s)')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  lookahead_min:      {lookahead_min:.2f}')
        self.get_logger().info(f'  lookahead_max:      {lookahead_max:.2f}')
        self.get_logger().info(f'  lookahead_k:        0.40')
        self.get_logger().info(f'  curvature_ff_gain:  {curvature_ff_gain:.2f}')
        self.get_logger().info(f'  cross_track_gain:   {cross_track_gain:.1f}')
        self.get_logger().info(f'  heading_gain:       {heading_gain:.1f}')
        self.get_logger().info(f'  max_omega:          {max_omega:.1f}')
        self.get_logger().info('=' * 60)

        # ── 输出可直接复制到 mission.launch.py 的片段 ──
        print(f'''
# === 复制以下到 mission.launch.py path_follower parameters ===
'lookahead_min': {lookahead_min:.2f},
'lookahead_max': {lookahead_max:.2f},
'lookahead_k': 0.40,
'curvature_ff_gain': {curvature_ff_gain:.2f},
'cross_track_gain': {cross_track_gain:.1f},
'heading_gain': {heading_gain:.1f},
'max_omega': {max_omega:.1f},
# ============================================================
''')


def main():
    rclpy.init()
    node = StepTuner()

    # 前置零速 2 秒
    node.get_logger().info('发送零速 2s, 确保底盘就绪...')
    node._send_cmd(0.0, 0.0)
    time.sleep(2.0)

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('中断, 停车')
    finally:
        node._send_cmd(0.0, 0.0)
        time.sleep(0.3)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
