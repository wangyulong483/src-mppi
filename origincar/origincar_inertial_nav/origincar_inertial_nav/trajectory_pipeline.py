#!/usr/bin/env python3
"""
trajectory_pipeline.py -- 多圈轨迹后处理: 中值中心线提取 + 通道内外分析
================================================================
把 path_recorder 录的 *多圈* CSV 用统计法提炼成一条贴通道中心的最优参考线,
抵消单圈手抖误差, 并量化"黄色通道内外"压线风险(rule.md 子任务2)。

流程 (对应用户 5 步):
  一 多圈录制    : path_recorder 每圈存一个 path_*.csv(本脚本读入, 不负责录)
  二 统一坐标系  : 所有圈天然共享同一 odom 系(每圈从 P 点 odom 归零启动),
                   无需对齐; 只在"分析/画图"时刚体配准到场地系(field_model)
  三 提取中心线  : 横向中值法(Cross-Sectional Median, Chen&Krumm 2010)——
                   各圈按弧长归一重采样→逐站取跨圈横向偏移中值→抗手抖中心线
  五 通道内外分析: 把原始点配准到场地系, 用 field_model 判内外, 出出界率报告+图

设计要点:
  · 中心线在 *原生 odom 系* 提取, 输出 CSV 零变换即可喂 path_follower。
  · 场地配准只影响分析图的绝对准度, 不影响中心线 —— 配准失败也不影响交付。

不依赖 ROS。用法:
  python3 trajectory_pipeline.py <laps_dir> -o path_ccw.csv --direction ccw \
          --plot ccw_analysis.png
  ros2 run origincar_inertial_nav trajectory_pipeline ~/origincar_paths \
          -o config/path_ccw.csv --direction ccw --plot ccw.png

关键参数:
  laps_dir            多圈 CSV 目录 (读所有 path_*.csv)
  -o/--output         中心线输出 CSV (index,x,y,yaw,v; follower 直接用)
  --direction cw|ccw|auto  按 C 环绕行方向筛圈 (shoelace 符号判); auto=用多数派
  --stations N        中心线站点数 (弧长重采样, 默认 400)
  --spacing D         输出点间距 (m, 默认 0.05; 窄道建议 ≤0.05)
  --smooth W          中心线平滑窗口 (奇数, 默认 7; 0/1=不平滑)
  --channel MODE      通道几何来源:
                      centerline(默认) = 提取的中值中心线 ±halfwidth 的 0.5m 走廊
                                         (自洽, 不依赖场地绝对尺寸; 即 step3→step5 方法)
                      model            = field_model 里的绝对赛场环道几何(像素估值, 需自行校准)
  --halfwidth H       通道半宽 (m, 默认 0.25 = 0.5m 通道; 对应 rule "1/2 出界")
  --bc-arclen f0,f1   B+C 段弧长比例区间 (默认全取: 本包每圈从 T 型口起录, 本就是 B+C)
  --anchor px,py,deg  仅画图用: odom 原点在场地系的位姿 (m,m,deg)。本包惯导起点=T型口
                      ≈(2.5,2.5)m, 默认 2.5,2.5,90; 图整体转了就调 deg。
  --plot FILE         存内外分析 PNG
  --report FILE       存文本报告 (默认只打 stdout)
"""

import os
import csv
import glob
import math
import argparse
import numpy as np

# field_model 可能以 ROS 包(有前缀)或独立脚本(同目录)两种方式被导入
try:
    from origincar_inertial_nav.field_model import FieldModel, dist_points_to_polyline
except ImportError:
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from field_model import FieldModel, dist_points_to_polyline


# ── CSV 读写 (与 path_recorder / path_follower 契约一致: index,x,y,yaw,v) ──
def load_lap(path):
    """读一圈 CSV → (N,4) [x,y,yaw,v]。"""
    rows = []
    with open(path, 'r', newline='') as f:
        for row in csv.DictReader(f):
            rows.append([float(row['x']), float(row['y']),
                         float(row['yaw']), float(row.get('v', 0.0) or 0.0)])
    return np.asarray(rows, dtype=float)


def save_centerline(path, pts):
    """写中心线 CSV (index,x,y,yaw,v)。pts: (N,4)。"""
    os.makedirs(os.path.dirname(os.path.abspath(path)) or '.', exist_ok=True)
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['index', 'x', 'y', 'yaw', 'v'])
        for i, p in enumerate(pts):
            w.writerow([i, f"{p[0]:.4f}", f"{p[1]:.4f}",
                        f"{p[2]:.4f}", f"{p[3]:.4f}"])


def copy_origin_sidecar(src_csv, dst_csv):
    """若样板圈有 <csv>.origin(录制起点 map 位姿), 拷到中心线输出旁, 供 follower map 锚定。"""
    src = src_csv + '.origin'
    if os.path.exists(src):
        try:
            with open(src) as a, open(dst_csv + '.origin', 'w', newline='') as b:
                b.write(a.read())
            return True
        except OSError:
            pass
    return False


# ── 几何 ──────────────────────────────────────────────────────
def arclen_cum(P):
    """累计弧长 (N,)。P:(N,2)。"""
    d = np.linalg.norm(np.diff(P, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(d)])


def resample_by_arclen(P, n=None, spacing=None):
    """按弧长等间隔重采样。给 n 出 n 点; 给 spacing 出 total/spacing 点。P:(N,2)→(M,2)。"""
    s = arclen_cum(P)
    total = s[-1]
    if total < 1e-9:
        return P.copy()
    if n is None:
        n = max(2, int(round(total / max(spacing, 1e-3))) + 1)
    su = np.linspace(0.0, total, n)
    x = np.interp(su, s, P[:, 0])
    y = np.interp(su, s, P[:, 1])
    return np.column_stack([x, y]), su, s


def resample_series(vals, s_src, s_dst):
    """把随源弧长 s_src 的标量序列插到目标弧长 s_dst。"""
    return np.interp(s_dst, s_src, vals)


def shoelace_area(P):
    """有符号面积 (>0 逆时针 ccw, <0 顺时针 cw)。P:(N,2)。"""
    x, y = P[:, 0], P[:, 1]
    return 0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y)


def tangents_normals(R):
    """折线 R(N,2) 的单位切向/左法向。返回 (tang(N,2), normal(N,2))。"""
    d = np.gradient(R, axis=0)
    nrm = np.linalg.norm(d, axis=1, keepdims=True)
    nrm = np.where(nrm < 1e-9, 1e-9, nrm)
    t = d / nrm
    n = np.column_stack([-t[:, 1], t[:, 0]])   # 左法向
    return t, n


def smooth_xy(P, win):
    """居中滑动平均, 首/末固定。P:(N,2)→(N,2)。"""
    if win is None or win < 3:
        return P
    if win % 2 == 0:
        win += 1
    half = win // 2
    n = len(P)
    out = P.copy()
    for i in range(1, n - 1):
        lo, hi = max(0, i - half), min(n - 1, i + half)
        out[i] = P[lo:hi + 1].mean(axis=0)
    return out


def _local_radius(P):
    """每点局部曲率半径 (m), 三点法(Menger); 首末两点填 inf。P:(N,2)→(N,)。"""
    n = len(P)
    R = np.full(n, np.inf)
    for i in range(2, n - 2):
        a, b, c = P[i - 2], P[i], P[i + 2]
        ab = np.linalg.norm(a - b); bc = np.linalg.norm(b - c); ca = np.linalg.norm(c - a)
        area = abs((b[0]-a[0])*(c[1]-a[1]) - (c[0]-a[0])*(b[1]-a[1])) / 2.0
        R[i] = np.inf if area < 1e-9 else (ab * bc * ca) / (4.0 * area)
    return R


def enforce_min_radius(P, r_min, max_iter=400):
    """把曲率半径 < r_min 的紧弯局部圆整到 ≥ r_min, 使参考线可被阿克曼车跟踪。
    只在过紧处做温和拉普拉斯平滑(0.25/0.5/0.25), 直道/缓弯不动; 首末固定。
    原因: 横向中值提取会把弯角削得比任一原始圈更尖(实测 0.37m→0.21m), 低于车最小
    转弯半径 → 车转不过去而向外漂蹭外环。这里把弯角圆回车能跟的半径。"""
    if r_min <= 0:
        return P
    P = P.copy()
    n = len(P)
    for _ in range(max_iter):
        R = _local_radius(P)
        tight = np.where(R < r_min)[0]
        tight = tight[(tight > 0) & (tight < n - 1)]
        if tight.size == 0:
            break
        newP = P.copy()
        newP[tight] = 0.25 * P[tight - 1] + 0.5 * P[tight] + 0.25 * P[tight + 1]
        P = newP
    return P


def recompute_yaw_xy(P):
    """相邻点位置反算 yaw(N,)。末点沿用前一点。"""
    n = len(P)
    yaw = np.zeros(n)
    d = np.diff(P, axis=0)
    yaw[:-1] = np.arctan2(d[:, 1], d[:, 0])
    if n >= 2:
        yaw[-1] = yaw[-2]
    return yaw


# ── 中心线提取 (横向中值法) ───────────────────────────────────
def cross_sectional_median(laps_xy, stations):
    """各圈 (N_k,2) 列表 → 中心线骨架 (stations,2) + 每站 v 中值。
    横向中值: 逐站取跨圈"沿法向带符号横向偏移"的中值, 平移骨架点。"""
    res = []
    vser = []
    for lap in laps_xy:
        rs, su, s = resample_by_arclen(lap[:, :2], n=stations)
        res.append(rs)
        vser.append(resample_series(lap[:, 3], s, su))
    res = np.stack(res)                       # (K, stations, 2)
    R = np.median(res, axis=0)                # 逐分量中值骨架
    _, nrm = tangents_normals(R)
    diff = res - R[None]                       # (K, stations, 2)
    off = np.einsum('ksi,si->ks', diff, nrm)   # 各圈各站横向偏移
    s_med = np.median(off, axis=0)             # 中值偏移 (stations,)
    center = R + s_med[:, None] * nrm
    v_med = np.median(np.stack(vser), axis=0)  # 每站 v 中值
    return center, v_med, res


# ── 方向分组 ──────────────────────────────────────────────────
def detect_direction(lap_xy):
    return 'ccw' if shoelace_area(lap_xy[:, :2]) > 0 else 'cw'


# ── 场地配准 (仅供分析/画图; 刚体 odom→场地, 靠"最大化通道内点率"自校准) ──
def apply_xform(P, theta, tx, ty):
    c, s = math.cos(theta), math.sin(theta)
    Rt = np.array([[c, -s], [s, c]])
    return P @ Rt.T + np.array([tx, ty])


def register_to_field(points_odom, fm, anchor, coarse_only=False):
    """搜刚体 (theta,tx,ty) 使 points 落进黄色通道的比例最大。
    anchor=(px,py,deg): odom 原点(录制起点=T型口)的场地位姿。
    odom 原点↔锚点是硬对应, 故平移锁在锚点附近小范围, 主要求航向 theta。
    返回 (theta,tx,ty,inside_rate)。"""
    px, py, deg = anchor
    h0 = math.radians(deg)
    # 子采样加速
    P = points_odom
    if len(P) > 800:
        P = P[np.random.default_rng(0).choice(len(P), 800, replace=False)]

    def scan(thetas, txs, tys, Q):
        best = (-1.0, h0, px, py)
        for th in thetas:
            for tx in txs:
                for ty in tys:
                    F = apply_xform(Q, th, tx, ty)
                    r = float(np.mean(fm.inside_channel(F)))
                    if r > best[0]:
                        best = (r, th, tx, ty)
        return best
    # 粗搜: theta 全周(锚点已定平移, 只求航向), 平移仅在锚点附近小范围(吸收"大致"误差)
    thetas = h0 + np.radians(np.arange(-180, 180, 10))
    txs = px + np.linspace(-0.25, 0.25, 7)
    tys = py + np.linspace(-0.25, 0.25, 7)
    r, th, tx, ty = scan(thetas, txs, tys, P)
    if coarse_only:
        return th, tx, ty, r
    # 精搜: 只用当前落在 B+C(场地 y≥A顶) 的点评分, 剔除 A 区恒外点的干扰 → 锁紧位姿
    for _ in range(2):
        F = apply_xform(P, th, tx, ty)
        sub = P[F[:, 1] >= fm.a_top]
        if len(sub) < 20:
            sub = P
        thetas = th + np.radians(np.linspace(-12, 12, 13))
        txs = tx + np.linspace(-0.2, 0.2, 9)
        tys = ty + np.linspace(-0.2, 0.2, 9)
        r, th, tx, ty = scan(thetas, txs, tys, sub)
    return th, tx, ty, r


# ── B+C 切段 ──────────────────────────────────────────────────
def split_bc_indices(lap_xy, bc_arclen=None):
    """返回 B+C 段布尔掩码。本包每圈从 T 型口起录, 本身即 B+C, 默认全取;
    bc_arclen=(f0,f1) 时按弧长比例手动切(录了 A 区引入段时用)。"""
    n = len(lap_xy)
    if bc_arclen is None:
        return np.ones(n, dtype=bool)
    s = arclen_cum(lap_xy[:, :2])
    f = s / max(s[-1], 1e-9)
    return (f >= bc_arclen[0]) & (f <= bc_arclen[1])


# ── 通道带几何 (通道=中心线±hw) ──────────────────────────────
def channel_band_polygon(center_xy, halfwidth):
    """中心线两侧各偏移 halfwidth, 拼成 0.5m 走廊多边形 (供画黄色带)。"""
    _, nrm = tangents_normals(center_xy)
    left = center_xy + nrm * halfwidth
    right = center_xy - nrm * halfwidth
    return np.vstack([left, right[::-1]])


# ── 画图 (通道=中心线±hw): 区底色背景 + 中心线走廊带 + 内外散点 ─
def make_plot(out_png, fm, xform, laps_xy, center_xy, inside_masks, halfwidth):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    fig, ax = plt.subplots(figsize=(8, 8))
    fm.draw_regions(ax)                       # A/B/C 区底色背景
    C = apply_xform(center_xy, *xform)
    band = apply_xform(channel_band_polygon(center_xy, halfwidth), *xform)
    ax.add_patch(mpatches.Polygon(band, closed=True, facecolor='#ffe14d',
                                  edgecolor='none', alpha=0.9, zorder=1))
    for lap, inside in zip(laps_xy, inside_masks):
        F = apply_xform(lap[:, :2], *xform)
        ax.scatter(F[~inside, 0], F[~inside, 1], s=4, c='#1f77b4', zorder=3)
        ax.scatter(F[inside, 0], F[inside, 1], s=4, c='#ff7f0e', zorder=3)
    ax.plot(C[:, 0], C[:, 1], '-', c='#d62728', lw=2.0, zorder=4)
    ax.scatter([], [], s=16, c='#1f77b4', label='outside yellow channel')
    ax.scatter([], [], s=16, c='#ff7f0e', label='inside yellow channel')
    ax.plot([], [], '-', c='#d62728', lw=2.0, label='median centerline')
    ax.set_title('Points Inside Yellow Channel')
    ax.legend(loc='lower right', markerscale=1)
    fig.savefig(out_png, dpi=120, bbox_inches='tight')
    plt.close(fig)


def anchor_xform(anchor):
    """确定性 odom→场地刚体变换 (仅画图定向): field = R(deg)·odom + (px,py)。"""
    px, py, deg = anchor
    return (math.radians(deg), px, py)


# ── 主流程 ────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(
        description='多圈轨迹→中值中心线提取+通道内外分析')
    ap.add_argument('laps_dir', help='多圈 CSV 目录 (path_*.csv)')
    ap.add_argument('-o', '--output', required=True, help='中心线输出 CSV')
    ap.add_argument('--direction', default='auto', choices=['cw', 'ccw', 'auto'])
    ap.add_argument('--stations', type=int, default=400)
    ap.add_argument('--spacing', type=float, default=0.05)
    ap.add_argument('--smooth', type=int, default=7)
    ap.add_argument('--min-radius', type=float, default=0.40,
                    help='最小曲率半径 m (默认 0.40; 把过紧弯角圆整到车能跟的半径, '
                         '阿克曼车 R_min≈0.31m; 0=关。中值提取会削尖弯角故需此步)')
    ap.add_argument('--channel', default='centerline',
                    choices=['centerline', 'model'],
                    help='通道几何: centerline=中心线±halfwidth(默认); model=赛场绝对环道')
    ap.add_argument('--halfwidth', type=float, default=0.25,
                    help='通道半宽 m (默认 0.25=0.5m 通道)')
    ap.add_argument('--bc-arclen', default=None,
                    help='B+C 段弧长比例 "f0,f1" (默认全取; 录了 A 区引入段时用)')
    ap.add_argument('--anchor', default='2.5,2.5,90',
                    help='仅画图定向: odom 原点(=T型口)场地位姿 px,py,deg')
    ap.add_argument('--plot', default=None)
    ap.add_argument('--report', default=None)
    args = ap.parse_args()

    laps_dir = os.path.expanduser(args.laps_dir)
    files = sorted(glob.glob(os.path.join(laps_dir, 'path_*.csv')))
    files = [f for f in files if not f.endswith('.origin')]
    if not files:
        print(f"[x] {laps_dir} 下没有 path_*.csv")
        return
    laps = [(f, load_lap(f)) for f in files]
    laps = [(f, l) for f, l in laps if len(l) >= 5]
    print(f"载入 {len(laps)} 圈: " + ', '.join(os.path.basename(f) for f, _ in laps))

    # 方向分组
    dirs = [detect_direction(l) for _, l in laps]
    if args.direction == 'auto':
        want = max(set(dirs), key=dirs.count)
        print(f"auto 方向 = 多数派 '{want}' ({dirs.count(want)}/{len(dirs)})")
    else:
        want = args.direction
    keep = [(f, l) for (f, l), d in zip(laps, dirs) if d == want]
    if not keep:
        print(f"[x] 没有方向为 {want} 的圈 (检测到: {dirs})")
        return
    if len(keep) < len(laps):
        print(f"按方向 {want} 保留 {len(keep)}/{len(laps)} 圈")

    fm = FieldModel()
    hw = args.halfwidth
    anchor = tuple(float(x) for x in args.anchor.split(','))

    # B+C 切段 (本包每圈从 T 型口起录, 默认全取; --bc-arclen 手动切)
    bc_arclen = None
    if args.bc_arclen:
        bc_arclen = tuple(float(x) for x in args.bc_arclen.split(','))
    laps_bc = []
    for f, l in keep:
        seg = l[split_bc_indices(l, bc_arclen)]
        if len(seg) >= 5:
            laps_bc.append((f, seg))
    if not laps_bc:
        print("[x] 切段后无有效点")
        return
    laps_xy = [s for _, s in laps_bc]

    # 横向中值中心线 (原生 odom 系)
    center, v_med, _ = cross_sectional_median(laps_xy, args.stations)
    center = smooth_xy(center, args.smooth)
    r0 = _local_radius(center).min()
    center = enforce_min_radius(center, args.min_radius)
    if args.min_radius > 0:
        r1 = _local_radius(center).min()
        print(f"最小曲率半径: {r0:.3f}m -> {r1:.3f}m "
              f"(圆整到 ≥{args.min_radius}m, 车 R_min≈0.31m 才跟得住)")
    rs, su, _ = resample_by_arclen(center, spacing=args.spacing)
    v_out = resample_series(v_med, arclen_cum(center), su)
    out_pts = np.column_stack([rs, recompute_yaw_xy(rs), v_out])

    save_centerline(args.output, out_pts)
    had_origin = copy_origin_sidecar(laps_bc[0][0], args.output)
    print(f"[OK] 中心线 -> {args.output}  ({len(out_pts)} 点, "
          f"长 {arclen_cum(rs)[-1]:.2f}m, 间距 {args.spacing}m"
          f"{', 含 .origin 边车' if had_origin else ''})")

    # ── 内外判定 ────────────────────────────────────────────
    # centerline 模式(默认): inside = 到提取中心线距离 ≤ halfwidth (原生 odom, 无需配准)
    # model 模式          : inside = field_model 绝对环道 (需 --anchor 配准, 已弃用为选配)
    if args.channel == 'centerline':
        def classify(P):
            return dist_points_to_polyline(P[:, :2], rs) <= hw
    else:
        th, tx, ty, _ = register_to_field(
            np.vstack(laps_xy)[:, :2], fm, anchor)
        anchor = (0, 0, 0)  # 占位; model 模式画图用 register 结果
        model_xform = (th, tx, ty)

        def classify(P):
            return fm.inside_channel(apply_xform(P[:, :2], *model_xform))

    per_lap, masks = [], []
    for f, seg in laps_bc:
        m = classify(seg)
        masks.append(m)
        per_lap.append((os.path.basename(f), float(np.mean(m)), len(seg)))
    all_rate = float(np.mean(np.concatenate(masks)))
    c_rate = float(np.mean(classify(out_pts)))

    lines = [f"=== 通道内外分析 (通道={args.channel}, 半宽 {hw}m = {2*hw}m 走廊) ==="]
    lines.append(f"原始多圈总体通道内点率: {all_rate*100:.1f}%  (出界 {100-all_rate*100:.1f}%)")
    lines.append(f"中值中心线通道内点率  : {c_rate*100:.1f}%")
    lines.append("每圈通道内点率(低=该圈手抖大, 偏离理想线多):")
    for name, rate, npt in sorted(per_lap, key=lambda x: x[1]):
        lines.append(f"  {name:32s} {rate*100:5.1f}%  ({npt} 点)")
    worst = min(per_lap, key=lambda x: x[1])
    lines.append(f"最差圈: {worst[0]} ({worst[1]*100:.1f}%)")
    if args.channel == 'centerline':
        lines.append(f"[说明] 通道以中值中心线为轴、半宽 {hw}m。中心线内点率必=100%; "
                     f"每圈内点率反映该圈相对理想线的手抖幅度是否在 ±{hw}m 内。")
    report = '\n'.join(lines)
    print(report)
    if args.report:
        with open(os.path.expanduser(args.report), 'w') as f:
            f.write(report + '\n')
        print(f"报告 -> {args.report}")

    # 画图
    if args.plot:
        try:
            xform = (model_xform if args.channel == 'model'
                     else anchor_xform(anchor))
            make_plot(os.path.expanduser(args.plot), fm, xform,
                      laps_xy, rs, masks, hw)
            print(f"分析图 -> {args.plot}")
        except Exception as e:
            print(f"[!] 画图失败: {e}")


if __name__ == '__main__':
    main()
