#!/usr/bin/env python3
"""Independent fullscreen subscriber for QR and VLM result topics.

The node is deliberately read-only: it only subscribes to low-rate String
topics and never calls services or publishes control messages.  ROS callbacks
put data into a bounded queue; all Tk operations stay on the main thread.
"""

import os
import queue
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .formatting import parse_qr_result, parse_vlm_result


class ResultDisplayNode(Node):
    def __init__(self):
        super().__init__('result_screen_display')
        self.qr_topic = str(
            self.declare_parameter('qr_topic', '/qr_result').value)
        self.vlm_topic = str(
            self.declare_parameter('vlm_topic', '/vlm/result').value)
        self.fullscreen = bool(
            self.declare_parameter('fullscreen', True).value)
        self.hide_cursor = bool(
            self.declare_parameter('hide_cursor', True).value)
        self.display_server = str(
            self.declare_parameter('display_server', ':0').value)
        self.window_title = str(
            self.declare_parameter('window_title', 'OriginCar 识别结果').value)

        self.events = queue.Queue(maxsize=16)
        self.create_subscription(String, self.qr_topic, self._on_qr, 10)
        self.create_subscription(String, self.vlm_topic, self._on_vlm, 10)
        self.get_logger().info(
            f'屏幕显示节点就绪: QR={self.qr_topic}, VLM={self.vlm_topic}, '
            f'DISPLAY={self.display_server}')

    def _put_latest(self, item):
        try:
            self.events.put_nowait(item)
        except queue.Full:
            try:
                self.events.get_nowait()
            except queue.Empty:
                pass
            self.events.put_nowait(item)

    def _on_qr(self, msg):
        code, detail = parse_qr_result(msg.data)
        if code:
            self._put_latest(('qr', code, detail, time.time()))

    def _on_vlm(self, msg):
        caption, detail = parse_vlm_result(msg.data)
        if caption:
            self._put_latest(('vlm', caption, detail, time.time()))


class ResultScreen:
    BG = '#071525'
    PANEL = '#102840'
    QR = '#31d6a4'
    VLM = '#ffcc66'
    TEXT = '#f3f7fb'
    MUTED = '#9eb3c7'

    def __init__(self, root, node, tk):
        self.root = root
        self.node = node
        self.tk = tk
        self._fullscreen = node.fullscreen

        root.title(node.window_title)
        root.configure(bg=self.BG)
        root.attributes('-fullscreen', self._fullscreen)
        if node.hide_cursor:
            root.configure(cursor='none')
        root.bind('<Escape>', self._leave_fullscreen)
        root.bind('<F11>', self._toggle_fullscreen)
        root.protocol('WM_DELETE_WINDOW', self.close)

        root.grid_rowconfigure(0, weight=1)
        root.grid_rowconfigure(1, weight=3)
        root.grid_rowconfigure(2, weight=5)
        root.grid_columnconfigure(0, weight=1)

        self.header = tk.Label(
            root, text='ORIGINCAR  ·  识别结果', bg=self.BG,
            fg=self.TEXT, anchor='w', padx=24, pady=10)
        self.header.grid(row=0, column=0, sticky='nsew')

        self.qr_panel = tk.Frame(root, bg=self.PANEL, padx=24, pady=12)
        self.qr_panel.grid(row=1, column=0, sticky='nsew', padx=16, pady=(0, 8))
        self.qr_panel.grid_columnconfigure(1, weight=1)
        self.qr_title = tk.Label(
            self.qr_panel, text='扫码结果', bg=self.PANEL, fg=self.QR,
            anchor='w')
        self.qr_title.grid(row=0, column=0, sticky='w')
        self.qr_value = tk.Label(
            self.qr_panel, text='等待二维码…', bg=self.PANEL, fg=self.TEXT,
            anchor='w', justify='left')
        self.qr_value.grid(row=1, column=0, columnspan=2, sticky='nsew')
        self.qr_detail = tk.Label(
            self.qr_panel, text='', bg=self.PANEL, fg=self.MUTED,
            anchor='w', justify='left')
        self.qr_detail.grid(row=2, column=0, columnspan=2, sticky='ew')

        self.vlm_panel = tk.Frame(root, bg=self.PANEL, padx=24, pady=12)
        self.vlm_panel.grid(row=2, column=0, sticky='nsew', padx=16, pady=(0, 16))
        self.vlm_panel.grid_rowconfigure(1, weight=1)
        self.vlm_panel.grid_columnconfigure(0, weight=1)
        self.vlm_title = tk.Label(
            self.vlm_panel, text='图生文结果', bg=self.PANEL, fg=self.VLM,
            anchor='w')
        self.vlm_title.grid(row=0, column=0, sticky='w')
        self.vlm_value = tk.Label(
            self.vlm_panel, text='等待图像识别…', bg=self.PANEL, fg=self.TEXT,
            anchor='nw', justify='left')
        self.vlm_value.grid(row=1, column=0, sticky='nsew', pady=(8, 4))
        self.vlm_detail = tk.Label(
            self.vlm_panel, text='', bg=self.PANEL, fg=self.MUTED,
            anchor='w', justify='left')
        self.vlm_detail.grid(row=2, column=0, sticky='ew')

        root.update_idletasks()
        height = max(root.winfo_screenheight(), 480)
        width = max(root.winfo_screenwidth(), 800)
        base = max(14, min(30, height // 22))
        families = set(root.tk.call('font', 'families'))
        family = next((name for name in (
            'Noto Sans CJK SC', 'WenQuanYi Micro Hei', 'Microsoft YaHei',
            'Source Han Sans CN', 'DejaVu Sans') if name in families),
            root.tk.call('font', 'actual', 'TkDefaultFont', '-family'))
        self.header.configure(font=(family, base, 'bold'))
        self.qr_title.configure(font=(family, base, 'bold'))
        self.qr_value.configure(font=(family, base * 2, 'bold'))
        self.qr_detail.configure(font=(family, max(12, base - 2)))
        self.vlm_title.configure(font=(family, base, 'bold'))
        self.vlm_value.configure(
            font=(family, max(16, int(base * 1.25))),
            wraplength=max(500, width - 96))
        self.vlm_detail.configure(font=(family, max(12, base - 2)))

        self.root.after(80, self._poll)

    @staticmethod
    def _clock(timestamp):
        return time.strftime('%H:%M:%S', time.localtime(timestamp))

    def _poll(self):
        if not rclpy.ok():
            self.close()
            return
        try:
            while True:
                kind, value, detail, timestamp = self.node.events.get_nowait()
                stamp = self._clock(timestamp)
                if kind == 'qr':
                    self.qr_value.configure(text=value)
                    self.qr_detail.configure(
                        text=f'{detail}    {stamp}' if detail else stamp)
                else:
                    self.vlm_value.configure(text=value)
                    self.vlm_detail.configure(
                        text=f'{detail}    {stamp}' if detail else stamp)
        except queue.Empty:
            pass
        self.root.after(80, self._poll)

    def _leave_fullscreen(self, _event=None):
        self._fullscreen = False
        self.root.attributes('-fullscreen', False)
        self.root.configure(cursor='')

    def _toggle_fullscreen(self, _event=None):
        self._fullscreen = not self._fullscreen
        self.root.attributes('-fullscreen', self._fullscreen)
        if self.node.hide_cursor:
            self.root.configure(cursor='none' if self._fullscreen else '')

    def close(self):
        try:
            self.root.destroy()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ResultDisplayNode()

    # 明确指向车载桌面，避免通过 SSH 启动时误开到转发的远端 DISPLAY。
    if node.display_server:
        os.environ['DISPLAY'] = node.display_server

    try:
        import tkinter as tk
        root = tk.Tk()
    except Exception as exc:
        node.get_logger().error(
            f'无法打开车载屏幕，显示节点将独立退出（其他节点不受影响）: {exc}')
        node.destroy_node()
        rclpy.shutdown()
        return

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    ResultScreen(root, node, tk)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)
        node.destroy_node()


if __name__ == '__main__':
    main()
