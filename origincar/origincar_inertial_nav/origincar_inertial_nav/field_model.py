#!/usr/bin/env python3
"""
field_model.py -- 竞赛场地黄色通道几何模型 (段2 惯导后处理流水线共用库)
================================================================
把 rule.md + 场地示意图(A6FC58087050AA847219281112DDE847.png)里的 cm 标注
落成一个可编程的场地模型, 供 trajectory_pipeline 做"通道内外分析"和画图。

坐标系 (场地系, 与 odom 系区分):
  原点 = 场地左下角, x→右, y→上, 单位 m。总场 5.0m × 5.0m。
  区域 (rule.md:47-64):
    A 区(大厅, 浅蓝)  : y ∈ [0, 2.0]      —— 规则允许任意行驶, 不做通道约束
    B 区(通道区, 浅灰): y ∈ [2.0, 2.5]     —— 只能走黄色通道
    C 区(诊疗区, 浅绿): y ∈ [2.5, 5.0]     —— 只能走黄色环道

黄色通道 (车身 1/2 出界即判出, rule.md:212):
  · B 通道: "长 0.5m、宽 1m"(rule.md:55) —— 竖直连接 A/C, 横向 1m 宽(半宽 0.5m),
            纵向占满 B 区 0.5m。故 B 段较宽松。
  · C 环道: "通道宽度 0.5m"(rule.md:63) —— 圆角矩形环, 半宽 0.25m。
  两段半宽不同, inside 判定按所属段各自的半宽。

注意: 图中像素难精确到 cm, 所有尺寸都作为 FieldModel 字段暴露, 可按实测微调。
本模型只用于"分析/画图", 不参与中心线提取与 path_follower(它们走原生 odom 系)。

自检:  python3 field_model.py   # 画场地 + 一批合成点内外着色, 存 field_model_selftest.png
"""

from dataclasses import dataclass
import numpy as np


# ── 几何工具 ──────────────────────────────────────────────────
def rounded_rect(xl, xr, yb, yt, r, n_arc=16):
    """逆时针生成圆角矩形折线 (闭合, 首=尾), 返回 (N,2)。
    xl/xr/yb/yt = 左右下上边界; r = 圆角半径。"""
    r = max(0.0, min(r, (xr - xl) / 2.0, (yt - yb) / 2.0))
    pts = [(xl + r, yb), (xr - r, yb)]                    # 下边
    corners = [
        (xr - r, yb + r, -np.pi / 2, 0.0),                # 右下角
        (xr - r, yt - r, 0.0, np.pi / 2),                 # 右上角
        (xl + r, yt - r, np.pi / 2, np.pi),               # 左上角
        (xl + r, yb + r, np.pi, 1.5 * np.pi),             # 左下角
    ]
    for cx, cy, a0, a1 in corners:
        for a in np.linspace(a0, a1, n_arc):
            pts.append((cx + r * np.cos(a), cy + r * np.sin(a)))
    pts.append(pts[0])                                     # 闭合
    return np.asarray(pts, dtype=float)


def dist_points_to_polyline(P, poly):
    """每个查询点到折线的最近距离 (向量化)。
    P: (K,2) 查询点; poly: (M,2) 折线顶点。返回 (K,)。"""
    P = np.atleast_2d(np.asarray(P, dtype=float))
    A = poly[:-1]                      # (S,2) 段起点
    B = poly[1:]                       # (S,2) 段终点
    AB = B - A                         # (S,2)
    ab2 = np.einsum('ij,ij->i', AB, AB)
    ab2 = np.where(ab2 < 1e-12, 1e-12, ab2)
    # t = clamp(((P-A)·AB)/|AB|^2, 0, 1); 广播 (K,S)
    AP = P[:, None, :] - A[None, :, :]                 # (K,S,2)
    t = np.einsum('ksj,sj->ks', AP, AB) / ab2[None, :]
    t = np.clip(t, 0.0, 1.0)
    proj = A[None, :, :] + t[:, :, None] * AB[None, :, :]   # (K,S,2)
    d = np.linalg.norm(P[:, None, :] - proj, axis=2)        # (K,S)
    return d.min(axis=1)


# ── 场地模型 ──────────────────────────────────────────────────
@dataclass
class FieldModel:
    # 总场 (m)
    width: float = 5.0
    height: float = 5.0
    # 区域分界 (y)
    a_top: float = 2.0          # A 顶 = B 底
    b_top: float = 2.5          # B 顶 = C 底
    # B 通道 (竖直, 宽 1m 居中): x ∈ [b_cx±b_halfwidth], y ∈ [a_top, b_top]
    b_cx: float = 2.5
    b_halfwidth: float = 0.5
    # C 环道中心线矩形 (mid-channel, 圆角), 通道半宽 0.25m
    ring_x_left: float = 1.25
    ring_x_right: float = 3.75
    ring_y_bottom: float = 2.75
    ring_y_top: float = 4.25
    ring_corner_r: float = 0.35
    ring_halfwidth: float = 0.25
    _ring_poly: np.ndarray = None    # 懒缓存

    # ── 通道中心线 ────────────────────────────────────────────
    def ring_centerline(self):
        """C 环道中心线折线 (N,2), 场地系。"""
        if self._ring_poly is None:
            self._ring_poly = rounded_rect(
                self.ring_x_left, self.ring_x_right,
                self.ring_y_bottom, self.ring_y_top, self.ring_corner_r)
        return self._ring_poly

    def b_centerline(self):
        """B 通道中心线 (竖直段, x=b_cx)。"""
        return np.array([[self.b_cx, self.a_top], [self.b_cx, self.b_top]])

    def channel_centerline(self):
        """B + C 合并中心线 (仅供画/参考), dict。"""
        return {'b': self.b_centerline(), 'c': self.ring_centerline()}

    # ── 内外判定 ──────────────────────────────────────────────
    def inside_channel(self, P):
        """P: (K,2) 或 (2,) → bool (K,) 或标量。
        inside = 落在 B 矩形内 OR 到 C 环中心线距离 ≤ ring_halfwidth。"""
        P2 = np.atleast_2d(np.asarray(P, dtype=float))
        x, y = P2[:, 0], P2[:, 1]
        in_b = (np.abs(x - self.b_cx) <= self.b_halfwidth) & \
               (y >= self.a_top) & (y <= self.b_top)
        d_ring = dist_points_to_polyline(P2, self.ring_centerline())
        in_c = d_ring <= self.ring_halfwidth
        inside = in_b | in_c
        return inside if inside.size > 1 else bool(inside[0])

    # ── 画图: 只画 A/B/C 区底色 (供"通道=中心线±hw"模式当背景) ──
    def draw_regions(self, ax):
        import matplotlib.patches as mpatches
        ax.add_patch(mpatches.Rectangle((0, 0), self.width, self.a_top,
                     facecolor='#aec7e8', edgecolor='none', zorder=0))
        ax.add_patch(mpatches.Rectangle((0, self.a_top), self.width,
                     self.b_top - self.a_top, facecolor='#e8e0c0',
                     edgecolor='none', zorder=0))
        ax.add_patch(mpatches.Rectangle((0, self.b_top), self.width,
                     self.height - self.b_top, facecolor='#b5d6a7',
                     edgecolor='none', zorder=0))
        ax.set_xlim(-0.1, self.width + 0.1)
        ax.set_ylim(-0.1, self.height + 0.1)
        ax.set_aspect('equal')

    # ── 画图 (复现目标图风格; 用模型环道几何) ─────────────────
    def draw_field(self, ax):
        """在 ax 上画区域底色 + 黄色通道带。仅示意, 不含尺寸标注。"""
        import matplotlib.patches as mpatches
        from matplotlib.path import Path
        from matplotlib.patches import PathPatch
        # 区域底色
        ax.add_patch(mpatches.Rectangle((0, 0), self.width, self.a_top,
                     facecolor='#aec7e8', edgecolor='none', zorder=0))          # A 蓝
        ax.add_patch(mpatches.Rectangle((0, self.a_top), self.width,
                     self.b_top - self.a_top, facecolor='#e8e0c0',
                     edgecolor='none', zorder=0))                                # B 灰黄
        ax.add_patch(mpatches.Rectangle((0, self.b_top), self.width,
                     self.height - self.b_top, facecolor='#b5d6a7',
                     edgecolor='none', zorder=0))                                # C 绿
        # 黄色 B 通道矩形
        ax.add_patch(mpatches.Rectangle(
            (self.b_cx - self.b_halfwidth, self.a_top),
            2 * self.b_halfwidth, self.b_top - self.a_top,
            facecolor='#ffe14d', edgecolor='none', zorder=1))
        # 黄色 C 环带 = 外圆角矩形填黄, 内圆角矩形挖回绿色
        hw = self.ring_halfwidth
        outer = rounded_rect(self.ring_x_left - hw, self.ring_x_right + hw,
                             self.ring_y_bottom - hw, self.ring_y_top + hw,
                             self.ring_corner_r + hw)
        inner = rounded_rect(self.ring_x_left + hw, self.ring_x_right - hw,
                             self.ring_y_bottom + hw, self.ring_y_top - hw,
                             max(self.ring_corner_r - hw, 0.02))
        band = Path(np.vstack([outer, inner[::-1]]),
                    [Path.MOVETO] + [Path.LINETO] * (len(outer) - 1) +
                    [Path.MOVETO] + [Path.LINETO] * (len(inner) - 1))
        ax.add_patch(PathPatch(band, facecolor='#ffe14d', edgecolor='none',
                               zorder=1))
        ax.set_xlim(-0.1, self.width + 0.1)
        ax.set_ylim(-0.1, self.height + 0.1)
        ax.set_aspect('equal')


# ── 自检 ──────────────────────────────────────────────────────
def _selftest():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fm = FieldModel()
    rng = np.random.default_rng(0)
    # 沿 B+C 通道中心线附近撒点 + 一些离散噪声点
    ring = fm.ring_centerline()
    idx = rng.integers(0, len(ring) - 1, 2000)
    pts = ring[idx] + rng.normal(0, 0.18, (2000, 2))
    bpts = np.column_stack([
        fm.b_cx + rng.normal(0, 0.35, 400),
        rng.uniform(fm.a_top, fm.b_top, 400)])
    allpts = np.vstack([pts, bpts])
    inside = fm.inside_channel(allpts)

    fig, ax = plt.subplots(figsize=(7, 7))
    fm.draw_field(ax)
    ax.scatter(allpts[~inside, 0], allpts[~inside, 1], s=4, c='#1f77b4',
               label='outside yellow channel', zorder=3)
    ax.scatter(allpts[inside, 0], allpts[inside, 1], s=4, c='#ff7f0e',
               label='inside yellow channel', zorder=3)
    ax.set_title('field_model self-test (Points Inside Yellow Channel)')
    ax.legend(loc='lower right', markerscale=3)
    out = 'field_model_selftest.png'
    fig.savefig(out, dpi=110, bbox_inches='tight')
    print(f"内点 {int(inside.sum())}/{len(allpts)} ({inside.mean()*100:.1f}%), "
          f"自检图 -> {out}")


if __name__ == '__main__':
    _selftest()
