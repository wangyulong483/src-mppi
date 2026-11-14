#!/usr/bin/env python3
"""
vlm_node.py -- 段2 VLM 图生文节点 (阿里云百炼 Qwen3.7-Plus, OpenAI 兼容)
=========================================================================
两种触发模式:
  waypoint — 订阅 /path_follower/waypoint_index, 到达指定路点索引时抓拍 (比赛模式)
  manual   — 终端按 Enter 键抓拍, 调用云端 API 并在终端打印结果 (调试模式)

门控:  ~/enable 服务 (std_srvs/SetBool) — 仅 waypoint 模式有效; manual 模式始终就绪

API: DashScope OpenAI兼容接口 (base64 JPEG → qwen3.7-plus)

用法:
  ros2 run origincar_vlm vlm_node --ros-args -p trigger_mode:=manual
  ros2 launch origincar_vlm vlm.launch.py  (调试模式, 自动启动深度相机)
"""

import base64
import json
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor

from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
from std_srvs.srv import SetBool, Trigger


class VlmNode(Node):
    def __init__(self):
        super().__init__('vlm_node')

        # ── API 参数 ──────────────────────────────────────────
        self.api_key = self.declare_parameter('api_key', '').value
        if not self.api_key:
            self.api_key = os.environ.get('DASHSCOPE_API_KEY', '')
        self.model_id = self.declare_parameter(
            'model_id', 'qwen3.7-plus').value
        self.endpoint = self.declare_parameter(
            'endpoint',
            'https://llm-cjkdp6ay5b8gfn8s.cn-beijing.maas.aliyuncs.com/compatible-mode/v1/chat/completions').value
        self.max_tokens = self.declare_parameter('max_tokens', 1024).value
        self.temperature = self.declare_parameter('temperature', 0.3).value
        self.timeout_sec = self.declare_parameter('timeout_sec', 30.0).value

        # ── 触发模式 ──────────────────────────────────────────
        self.trigger_mode = self.declare_parameter('trigger_mode', 'waypoint').value
        if self.trigger_mode not in ('waypoint', 'manual'):
            self.get_logger().warn(f'未知 trigger_mode={self.trigger_mode}, 回退到 waypoint')
            self.trigger_mode = 'waypoint'

        # ── waypoint 触发路点索引 (随扫码方向自动切换) ──────────
        # 段2方向由二维码决定(不是 launch 参数): stage2 收 /sign_switch_sign_data
        # (3=CW, 4=CCW) 切 path_follower 的 CSV。cw/ccw 路点编号方向相反, 立牌对应
        # 索引不同, 故本节点也订同一信号切触发集合, 保证与实际所跑路径一致。
        # 收到 QR 方向前默认用 cw 集合。
        def _parse_idx(raw):
            if isinstance(raw, str):
                raw = json.loads(raw) if raw else []
            elif not isinstance(raw, list):
                raw = []
            return set(int(x) for x in raw)
        # 空列表默认值会被 rclpy 推断为 BYTE_ARRAY (Parameter.Type.from_parameter_value([])
        # → BYTE_ARRAY), 导致 launch 传入的 INTEGER_ARRAY 覆盖被拒
        # (InvalidParameterTypeException, 节点崩溃)。描述符的 type 字段是"静态类型",
        # 不能改变由默认值推断出的类型 → 必须用 dynamic_typing=True 关闭类型检查,
        # 才允许覆盖值改变参数类型。
        _dyn = ParameterDescriptor(dynamic_typing=True)
        _idx_default = self.declare_parameter('trigger_indices', [56, 57]).value
        _idx_cw = self.declare_parameter('trigger_indices_cw', [], _dyn).value
        _idx_ccw = self.declare_parameter('trigger_indices_ccw', [], _dyn).value
        self._trigger_cw = _parse_idx(_idx_cw) or _parse_idx(_idx_default)
        self._trigger_ccw = _parse_idx(_idx_ccw) or self._trigger_cw
        self._direction = 'cw'
        self.trigger_indices = self._trigger_cw
        self.cooldown_sec = self.declare_parameter('cooldown_sec', 5.0).value
        self.vlm_prompt = self.declare_parameter(
            'prompt', '请详细描述这张图片中的内容，特别关注是否有人形立牌、人物模型或标识牌。'
                      '描述其外观特征、颜色、文字内容（如有）、姿态和位置。'
                      '立牌主题是关于医疗场景的，请尽可能详细，不少于100字。').value
        self.enable_debug = self.declare_parameter('enable_debug', False).value
        self.debug_dir = self.declare_parameter('debug_dir', '/tmp').value

        # ── 状态 ──────────────────────────────────────────────
        self._enabled = (self.trigger_mode == 'manual')  # manual 模式默认就绪
        self._latest_rgb = None
        self._latest_rgb_stamp = None
        self._rgb_lock = threading.Lock()
        self._last_trigger_time = 0.0
        self._triggered_indices = set()
        self._pending_inference = False
        self._running = True

        # ── 订阅 ──────────────────────────────────────────────
        if self.trigger_mode == 'waypoint':
            self.create_subscription(
                Int32, '/path_follower/waypoint_index',
                self._on_waypoint_index, 10)
            # QR 方向信号 (latched, 与 stage2 同源): 3=CW, 4=CCW → 切触发索引集合
            dir_qos = QoSProfile(
                depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST)
            self.create_subscription(
                Int32, '/sign_switch_sign_data', self._on_direction, dir_qos)

        # RGB 用 sensor QoS (best_effort), 兼容相机发布端
        rgb_qos = QoSProfile(
            depth=5,
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(
            Image, '/aurora/rgb/image_raw',
            self._on_rgb_image, rgb_qos)

        # ── 发布 ──────────────────────────────────────────────
        self.result_pub = self.create_publisher(String, '/vlm/result', 10)

        # ── 服务 ──────────────────────────────────────────────
        self.create_service(SetBool, '~/enable', self._on_enable)
        self.create_service(Trigger, '~/trigger', self._on_trigger_srv)

        # ── cv_bridge / cv2 (延迟导入) ────────────────────────
        self._cv_bridge = None
        self._cv2 = None
        self._requests = None

        # ── manual 模式: 启动键盘监听线程 ─────────────────────
        if self.trigger_mode == 'manual':
            self._kb_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
            self._kb_thread.start()

        self.get_logger().info(
            f'VLM 节点就绪: model={self.model_id} '
            f'mode={self.trigger_mode} '
            f'trigger_cw={sorted(self._trigger_cw)} '
            f'trigger_ccw={sorted(self._trigger_ccw)} (默认cw, 随QR切) '
            f'api_key={"已配置" if self.api_key else "⚠未配置"}')

        # 预热 HTTP 连接: 提前建立到百炼的 TCP+TLS, 首次推理无需等待握手
        if self.api_key and self.trigger_mode == 'waypoint':
            import requests as _req
            self._http = _req.Session()
            self._http.headers.update({
                'Content-Type': 'application/json',
                'Authorization': f'Bearer {self.api_key}',
            })
            try:
                # 用 /models 端点做轻量 HEAD 请求预热连接池
                base = self.endpoint.rsplit('/', 1)[0]  # strip /chat/completions
                self._http.head(base, timeout=5.0)
                self.get_logger().info('[VLM] HTTP 连接预热完成')
            except Exception:
                self.get_logger().warn('[VLM] HTTP 预热超时, 首次推理可能稍慢 (不影响功能)')

        if self.trigger_mode == 'manual':
            print('\n' + '=' * 60)
            print('  VLM 调试模式 — 按 Enter 拍照并调用云端大模型')
            print('  输入 q 或 Ctrl+C 退出')
            print('=' * 60 + '\n')

    # ── QR 方向信号: 切触发索引集合 (与 stage2 切路径同源同步) ──
    def _on_direction(self, msg):
        new_dir = 'ccw' if msg.data == 4 else 'cw'
        if new_dir == self._direction:
            return
        self._direction = new_dir
        self.trigger_indices = (self._trigger_ccw if new_dir == 'ccw'
                                else self._trigger_cw)
        self.get_logger().info(
            f'[QR] 方向={new_dir} (信号={msg.data}) → VLM 触发索引切为 '
            f'{sorted(self.trigger_indices)}')

    # ── 门控 (仅 waypoint 模式) ───────────────────────────────
    def _on_enable(self, req, resp):
        if self.trigger_mode == 'manual':
            resp.success = True
            resp.message = 'manual mode, always enabled'
            return resp
        self._enabled = req.data
        if self._enabled:
            self._triggered_indices.clear()
            self._last_trigger_time = 0.0
        resp.success = True
        resp.message = f"vlm_node {'enabled' if self._enabled else 'disabled'}"
        self.get_logger().info(f"[VLM] ~/enable: {resp.message}")
        return resp

    # ── 手动触发服务 ───────────────────────────────────────────
    def _on_trigger_srv(self, req, resp):
        """外部通过 ros2 service call 触发一次推理"""
        with self._rgb_lock:
            frame = self._latest_rgb.copy() if self._latest_rgb is not None else None
            stamp = self._latest_rgb_stamp

        if frame is None:
            resp.success = False
            resp.message = '尚无 RGB 帧, 请等待相机启动'
            return resp

        if self._pending_inference:
            resp.success = False
            resp.message = '上一次推理尚未完成'
            return resp

        self._pending_inference = True
        result = self._do_inference_sync('svc', frame, stamp)
        self._pending_inference = False

        resp.success = result is not None
        resp.message = result if result else '推理失败, 查看日志'
        return resp

    # ── 回调: 缓存最新 RGB 帧 ──────────────────────────────────
    def _on_rgb_image(self, msg):
        if self._cv_bridge is None:
            try:
                from cv_bridge import CvBridge
                self._cv_bridge = CvBridge()
            except ImportError:
                self.get_logger().error('cv_bridge 未安装, 无法解码 RGB 图像', throttle_duration_sec=10)
                return
        try:
            # bgr8 → numpy array (用 cv_bridge, 不是 cv2.imgmsg_to_cv2)
            np_arr = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self._rgb_lock:
                self._latest_rgb = np_arr
                self._latest_rgb_stamp = msg.header.stamp
        except Exception as e:
            self.get_logger().error(f'RGB 解码失败: {e}', throttle_duration_sec=10)

    # ── 回调: 路点索引 → 触发判断 ──────────────────────────────
    def _on_waypoint_index(self, msg):
        if not self._enabled:
            return
        idx = msg.data
        if idx not in self.trigger_indices:
            return
        if idx in self._triggered_indices:
            return    # 已触发过 (冷却窗口内不再重复)
        if self._pending_inference:
            return    # 上一帧还在推理中

        now = time.time()
        if now - self._last_trigger_time < self.cooldown_sec:
            return    # 冷却中

        self._triggered_indices.add(idx)
        self._last_trigger_time = now
        self.get_logger().info(f'[VLM] 触发: waypoint_index={idx} → 抓取 RGB 帧')

        # 异步推理 (不阻塞回调)
        with self._rgb_lock:
            frame = self._latest_rgb.copy() if self._latest_rgb is not None else None
            stamp = self._latest_rgb_stamp

        if frame is None:
            self.get_logger().warn(f'[VLM] 触发 idx={idx} 但尚无 RGB 帧, 跳过')
            return

        self._pending_inference = True
        thread = threading.Thread(
            target=self._do_inference,
            args=(idx, frame, stamp),
            daemon=True)
        thread.start()

    # ── 推理: 编码图像 → HTTP → 解析 (异步版, waypoint 用) ──
    def _do_inference(self, idx, bgr_frame, stamp):
        self._do_inference_sync(idx, bgr_frame, stamp)
        self._pending_inference = False

    # ── 推理: 同步版 (manual 模式用, 返回结果文本) ─────────────
    def _do_inference_sync(self, idx, bgr_frame, stamp) -> str | None:
        try:
            # 调试保存
            if self.enable_debug:
                if self._cv2 is None:
                    import cv2
                    self._cv2 = cv2
                import os as _os
                _os.makedirs(self.debug_dir, exist_ok=True)
                ts = time.strftime('%Y%m%d_%H%M%S', time.localtime())
                fname = _os.path.join(self.debug_dir, f'vlm_idx{idx}_{ts}.jpg')
                self._cv2.imwrite(fname, bgr_frame)
                self.get_logger().info(f'[VLM] 调试图片已保存: {fname}')

            # cv2 编码为 JPEG
            if self._cv2 is None:
                import cv2
                self._cv2 = cv2
            ok, jpeg_bytes = self._cv2.imencode('.jpg', bgr_frame,
                [self._cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ok:
                self.get_logger().error('[VLM] JPEG 编码失败')
                return None
            b64_img = base64.b64encode(jpeg_bytes.tobytes()).decode('utf-8')

            # 构造 OpenAI 兼容多模态请求
            payload = {
                'model': self.model_id,
                'max_tokens': self.max_tokens,
                'temperature': self.temperature,
                'messages': [{
                    'role': 'user',
                    'content': [
                        {'type': 'image_url',
                         'image_url': {'url': f'data:image/jpeg;base64,{b64_img}'}},
                        {'type': 'text', 'text': self.vlm_prompt},
                    ],
                }],
            }

            if self._requests is None:
                import requests
                self._requests = requests
            # 连接复用: Session 保持 TCP+TLS 长连接, 省去每次调用的握手开销
            if not hasattr(self, '_http'):
                self._http = requests.Session()
                self._http.headers.update({
                    'Content-Type': 'application/json',
                    'Authorization': f'Bearer {self.api_key}',
                })

            label = f'idx={idx}' if isinstance(idx, int) else idx
            self.get_logger().info(f'[VLM] {label} → 调用 {self.model_id}...')
            t0 = time.time()

            # 重试: 网络偶发抖动时再试 1 次, 间隔 3s
            last_err = None
            for attempt in range(2):
                try:
                    resp = self._http.post(
                        self.endpoint,
                        json=payload,
                        timeout=self.timeout_sec)
                    break
                except self._requests.exceptions.Timeout:
                    last_err = f'timeout ({self.timeout_sec}s)'
                    if attempt == 0:
                        self.get_logger().warn(
                            f'[VLM] {label} 第1次超时, 3s后重试...')
                        time.sleep(3.0)
                except self._requests.exceptions.ConnectionError as e:
                    last_err = f'connection: {e}'
                    if attempt == 0:
                        self.get_logger().warn(
                            f'[VLM] {label} 连接失败, 3s后重试: {e}')
                        time.sleep(3.0)
            else:
                self.get_logger().error(f'[VLM] {label} 2次尝试均失败: {last_err}')
                return last_err

            elapsed = time.time() - t0

            if resp.status_code != 200:
                err = f'API 返回 {resp.status_code}: {resp.text[:200]}'
                self.get_logger().error(f'[VLM] {err}')
                return err

            body = resp.json()
            result_text = (body.get('choices', [{}])[0]
                           .get('message', {})
                           .get('content', ''))

            if not result_text:
                self.get_logger().warn(
                    f'[VLM] API 返回空: {json.dumps(body, ensure_ascii=False)[:300]}')
                return None

            self.get_logger().info(
                f'[VLM] {label} ✓ ({elapsed:.1f}s): {result_text[:120]}')

            # 发布到 /vlm/result
            out = String()
            out.data = json.dumps({
                'index': idx if isinstance(idx, int) else -1,
                'text': result_text,
                'elapsed_s': round(elapsed, 2),
                'model': self.model_id,
            }, ensure_ascii=False, indent=2)
            self.result_pub.publish(out)

            return result_text

        except Exception as e:
            self.get_logger().error(f'[VLM] 推理异常: {e}')
            return str(e)

    # ── 键盘监听循环 (manual 模式) ─────────────────────────────
    def _keyboard_loop(self):
        """独立线程: 轮询 stdin, 收到 Enter 时抓拍→推理→打印"""
        print('  等待按键: 按 Enter 拍照推理, 按 q 退出\n')

        # 尝试打开 /dev/tty 直读 (ROS2 launch 节点 stdin 可能已重定向)
        try:
            tty = open('/dev/tty', 'r')
        except (OSError, IOError):
            tty = sys.stdin

        while self._running:
            try:
                # 用 select 非阻塞轮询, 每次等 0.3s
                import select as sel
                if hasattr(tty, 'fileno'):
                    r, _, _ = sel.select([tty], [], [], 0.3)
                    if not r:
                        continue
                line = tty.readline()
                if not line:
                    continue

                line = line.strip()
                if line.lower() == 'q':
                    print('\n退出调试模式.')
                    self._running = False
                    rclpy.shutdown()
                    break
                if line != '':
                    continue  # 只响应空行 (裸 Enter)

                # 抓帧
                with self._rgb_lock:
                    frame = self._latest_rgb.copy() if self._latest_rgb is not None else None
                    stamp = self._latest_rgb_stamp

                if frame is None:
                    print('[!] 尚无相机画面, 请等待深度相机启动 (约3-5秒)\n')
                    continue

                if self._pending_inference:
                    print('[!] 上一次推理尚未完成, 请稍候...\n')
                    continue

                self._pending_inference = True
                print(f'📷 已抓拍 @ {stamp.sec}.{stamp.nanosec:09d}')
                print(f'⏳ 调用 {self.model_id}...')
                t0 = time.time()
                result = self._do_inference_sync('manual', frame, stamp)
                dt = time.time() - t0
                self._pending_inference = False

                if result and not result.startswith('API 返回'):
                    print(f'\n{"─" * 50}')
                    print(f'🤖 VLM 结果 ({dt:.1f}s):')
                    print(f'   {result}')
                    print(f'{"─" * 50}\n')
                else:
                    print(f'❌ 推理失败 ({dt:.1f}s): {result}\n')

            except (EOFError, KeyboardInterrupt):
                self._running = False
                break
            except Exception as e:
                print(f'[!] 异常: {e}\n')
                self._pending_inference = False


def main(args=None):
    rclpy.init(args=args)
    node = VlmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
