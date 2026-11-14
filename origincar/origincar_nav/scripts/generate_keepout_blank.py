#!/usr/bin/env python3
"""
生成空白 keepout mask (全 FREE), 覆盖在 room.pgm 同尺寸上。

用法:
  python3 generate_keepout_blank.py [输出路径]

默认覆盖 origincar_nav/maps/keepout_mask.pgm。
生成后直接 navigation.launch.py (use_keepout:=true) 即可跑——空白 mask 不挡任何位置。

建好大厅地图后, 算出 B/C 区在 mask 里的像素范围, 涂黑即变成禁区。
"""

import sys
from pathlib import Path

W, H = 163, 141  # 与 room.pgm 一致

out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).parent.parent / 'maps' / 'keepout_mask.pgm'
out.write_bytes(b'P5\n%d %d\n255\n' % (W, H) + b'\xff' * (W * H))
print(f'空白 keepout mask 已生成: {out} ({W}x{H}, all FREE)')
