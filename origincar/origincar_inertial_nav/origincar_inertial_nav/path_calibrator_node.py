#!/usr/bin/env python3
"""
path_calibrator.py -- 参考轨迹离线校准工具 (CSV 点位校准)
================================================================
对 path_recorder 录的 CSV 做几何校准, 让参考线更干净, 供 path_follower 追踪:

  1) 闭环纠偏 : 把"收尾误差"(末点 - 首点)按弧长比例沿全程分摊, 使首=尾。
                纯惯导跑一圈的漂移(本例 0.167m)是逐步累积的, 按弧长分摊最合理。
  2) 平滑     : 对内部点做居中滑动平均(首/末点固定, 不破坏闭环与起点)。
  3) 重算 yaw : 用相邻点位置反算航向 yaw=atan2(dy,dx), 比原始积分 yaw 更贴合几何。
  v 列保持原值(无时间信息不重算)。

不依赖 ROS, PC / 车上都能跑:
  ros2 run origincar_inertial_nav path_calibrator IN.csv -o OUT.csv --smooth 5
  # 或直接: python3 path_calibrator.py IN.csv -o OUT.csv

参数:
  input              输入 CSV (path_recorder 输出, 列 index,x,y,yaw,v)
  -o/--output        输出 CSV (默认 <input>_calib.csv)
  --smooth N         滑动平均窗口(奇数, 默认 5; 0/1=不平滑)
  --no-close         不做闭环纠偏(轨迹本就非闭环时用)
"""

import os
import csv
import math
import argparse


def load(path):
    pts = []
    with open(path, 'r', newline='') as f:
        for row in csv.DictReader(f):
            pts.append([float(row['x']), float(row['y']),
                        float(row['yaw']), float(row.get('v', 0.0) or 0.0)])
    return pts


def close_err(pts):
    return math.hypot(pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1])


def arclen(pts):
    """轨迹累计弧长 (m)。"""
    return sum(math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
               for i in range(1, len(pts)))


def loop_close(pts):
    """按累计弧长比例分摊收尾误差, 使末点=首点。首点不动。"""
    n = len(pts)
    if n < 3:
        return pts
    ex = pts[-1][0] - pts[0][0]
    ey = pts[-1][1] - pts[0][1]
    # 累计弧长
    s = [0.0] * n
    for i in range(1, n):
        s[i] = s[i - 1] + math.hypot(pts[i][0] - pts[i - 1][0],
                                     pts[i][1] - pts[i - 1][1])
    total = s[-1] if s[-1] > 1e-9 else 1.0
    for i in range(n):
        frac = s[i] / total
        pts[i][0] -= ex * frac
        pts[i][1] -= ey * frac
    return pts


def smooth(pts, win):
    """居中滑动平均, 只动内部点, 首/末固定。"""
    if win is None or win < 3:
        return pts
    if win % 2 == 0:
        win += 1
    half = win // 2
    n = len(pts)
    out = [p[:] for p in pts]
    for i in range(1, n - 1):
        lo, hi = max(0, i - half), min(n - 1, i + half)
        k = hi - lo + 1
        out[i][0] = sum(pts[j][0] for j in range(lo, hi + 1)) / k
        out[i][1] = sum(pts[j][1] for j in range(lo, hi + 1)) / k
    return out


def recompute_yaw(pts):
    """用相邻点位置反算 yaw。末点沿用前一点。"""
    n = len(pts)
    for i in range(n - 1):
        dx = pts[i + 1][0] - pts[i][0]
        dy = pts[i + 1][1] - pts[i][1]
        if abs(dx) > 1e-9 or abs(dy) > 1e-9:
            pts[i][2] = math.atan2(dy, dx)
    if n >= 2:
        pts[-1][2] = pts[-2][2]
    return pts


def save(path, pts):
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['index', 'x', 'y', 'yaw', 'v'])
        for i, p in enumerate(pts):
            w.writerow([i, f"{p[0]:.4f}", f"{p[1]:.4f}", f"{p[2]:.4f}", f"{p[3]:.4f}"])


def main():
    ap = argparse.ArgumentParser(description='参考轨迹离线校准: 闭环纠偏+平滑+重算yaw')
    ap.add_argument('input', help='输入 CSV (path_recorder 输出)')
    ap.add_argument('-o', '--output', default=None, help='输出 CSV (默认 <input>_calib.csv)')
    ap.add_argument('--smooth', type=int, default=5, help='滑动平均窗口(奇数, 0/1=不平滑)')
    ap.add_argument('--no-close', action='store_true', help='不做闭环纠偏')
    args = ap.parse_args()

    inp = os.path.expanduser(args.input)
    out = args.output or (os.path.splitext(inp)[0] + '_calib.csv')
    out = os.path.expanduser(out)

    pts = load(inp)
    if len(pts) < 3:
        print(f"点数太少({len(pts)}), 放弃")
        return
    err0 = close_err(pts)

    # ── 采样密度 + 平滑物理宽度自检 (窄道/小弯关键) ──────────────
    total0 = arclen(pts)
    avg_gap = total0 / (len(pts) - 1) if len(pts) > 1 else 0.0
    if args.smooth and args.smooth >= 3:
        win = args.smooth + 1 if args.smooth % 2 == 0 else args.smooth
        smooth_span = win * avg_gap
        print(f"平均点距 {avg_gap:.3f}m, 平滑窗口 {win} 点 = 物理宽度 {smooth_span:.3f}m")
        # 滑动平均把弧线往圆心拉(弦效应): 窗口物理宽度越大、弯半径越小, 参考线越被"平滑成切内弯"。
        if smooth_span > 0.30:
            print(f"  [!] 平滑宽度 {smooth_span:.2f}m 偏大: 小半径弯会被往内缩(弦效应), "
                  f"窄通道慎用。建议减小 --smooth 或用更密的 record_distance 重录。")

    # ── 开环 / 往返轨迹误闭环自检 ───────────────────────────────
    # 闭环纠偏假设首≈尾(同位置且同朝向)。两种情况都不该闭环:
    #   (a) 首尾相距占全程比例过大 → 明显开环;
    #   (b) 首尾位置接近但"首尾朝向相反"(>135°) → 往返轨迹(上入口直廊→绕环→原路下直廊),
    #       位置巧合重合但绝非闭环 —— 此时首尾距离占比很小, 光看距离会漏判。
    # 两种都应 --no-close; 否则闭环纠偏把末点硬拉到首点, 会在入口处扭出一个假弯。
    if not args.no_close:
        n = len(pts)
        th_s = math.atan2(pts[min(3, n - 1)][1] - pts[0][1],
                          pts[min(3, n - 1)][0] - pts[0][0])
        th_e = math.atan2(pts[-1][1] - pts[max(0, n - 4)][1],
                          pts[-1][0] - pts[max(0, n - 4)][0])
        dth = abs(math.atan2(math.sin(th_e - th_s), math.cos(th_e - th_s)))
        open_gap = total0 > 1e-6 and (err0 / total0) > 0.15
        head_rev = dth > 2.356      # 首尾朝向相反 >135° = 往返轨迹
        if open_gap or head_rev:
            why = []
            if open_gap:
                why.append(f"首尾相距占全程 {err0 / total0 * 100:.0f}%")
            if head_rev:
                why.append(f"首尾朝向相差 {math.degrees(dth):.0f}°(往返轨迹)")
            print(f"  [!] {', '.join(why)} —— 疑似非闭环, 强烈建议加 --no-close, "
                  f"否则闭环纠偏会在入口处扭曲形状。")

    if not args.no_close:
        pts = loop_close(pts)
    pts = smooth(pts, args.smooth)
    pts = recompute_yaw(pts)
    err1 = close_err(pts)

    save(out, pts)
    print(f"输入 {inp}  ({len(pts)} 点)")
    print(f"闭环误差: {err0:.3f}m -> {err1:.3f}m"
          f"{'' if args.no_close else ' (已分摊)'}, 平滑窗口={args.smooth}")
    print(f"输出 {out}")


if __name__ == '__main__':
    main()
