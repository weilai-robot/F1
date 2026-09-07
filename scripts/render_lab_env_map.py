#!/usr/bin/env python3
"""
render_lab_env_map.py — 将 CI 全局唯一地图 (lab_env.xml) 渲染为俯视图 PNG

与 gen_static_map.py 同源的几何解析 (geom_footprints + z_span 高度过滤):
  - 静态障碍按 MuJoCo material 配色 (wall/wood/metal/glass/chair/equip/carton/warning)
  - dyn_* 动态障碍单独配色并注明 "CI 中静态摆放"
  - 叠加 6 个场景 (A-F) 的目标链: 起点○ → 终点★ (直线仅示链序, 非实际规划路径)
  - 标注通道A(1.0m 可通行) / 通道B(0.35m 不可通行) / 玻璃开口(2.8m)

用法:
  python3 scripts/render_lab_env_map.py \
      --xml motion_control/module/sim_module/model/mjcf/environment/lab_env.xml \
      --out doc/lab_env_map.png
"""

import argparse
import math
import xml.etree.ElementTree as ET
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrow, Polygon, Rectangle
from matplotlib.lines import Line2D

# ── 中文字体 (macOS) ─────────────────────────────────────────
for _f in ["PingFang SC", "Hiragino Sans GB", "Arial Unicode MS", "Heiti TC"]:
    try:
        matplotlib.font_manager.findfont(_f, fallback_to_default=False)
        plt.rcParams["font.sans-serif"] = [_f] + plt.rcParams["font.sans-serif"]
        break
    except Exception:
        continue
plt.rcParams["axes.unicode_minus"] = False

# ── 材质 → 颜色 ──────────────────────────────────────────────
MAT_COLOR = {
    "wall":    ("#3a3f47", 1.0),   # 墙体/隔断/柱 深灰
    "wood":    ("#b08050", 1.0),   # 木质台面
    "metal":   ("#9aa0a8", 1.0),   # 金属台面/推车
    "chair":   ("#5b6b94", 0.95),  # 椅子 蓝灰
    "equip":   ("#4a4d52", 1.0),   # 设备机柜
    "carton":  ("#c8a066", 1.0),   # 纸箱 浅棕
    "warning": ("#f2c400", 1.0),   # 警示柱 黄
}
DYN_COLOR = {"dyn_person": "#e04a1f", "dyn_box": "#2b7fd4", "dyn_crate": "#b8a020"}

# ── 场景链 (与 nav_test_runner.py SCENARIOS 一致) ─────────────
SCENARIOS = [
    ("A_straight_5m",     (0.0, 0.0),  (5.0,  0.0),  "#2ca02c"),
    ("B_obstacle_bypass", (5.0, 0.0),  (8.0, -3.0),  "#1f77b4"),
    ("C_narrow_passage",  (8.0, -3.0), (-0.5, -3.0), "#d62728"),
    ("D_impassable",      (-0.5, -3.0), (5.0,  3.2), "#9467bd"),
    ("E_long_distance",   (5.0,  3.2), (0.0,  0.0),  "#8c564b"),
    ("F_return_trip",     (0.0,  0.0),  (5.0,  0.0),  "#17becf"),
]


def geom_footprints(root):
    """[(name, typ, material, rgba, size, world_pos)] — 复用 gen_static_map 的遍历"""
    out = []

    def walk(elem, base):
        for g in elem.findall("geom"):
            name = g.get("name", "")
            typ = g.get("type", "")
            mat = g.get("material", "")
            rgba = g.get("rgba", "")
            size = [float(v) for v in (g.get("size") or "").split() if v]
            pos = [float(v) for v in (g.get("pos") or "0 0 0").split()]
            pos = pos + [0.0] * (3 - len(pos))
            out.append((name, typ, mat, rgba, size,
                        (base[0] + pos[0], base[1] + pos[1], base[2] + pos[2])))
        for b in elem.findall("body"):
            p = [float(v) for v in (b.get("pos") or "0 0 0").split()]
            p = p + [0.0] * (3 - len(p))
            walk(b, (base[0] + p[0], base[1] + p[1], base[2] + p[2]))

    walk(root, (0.0, 0.0, 0.0))
    return out


def z_span(name, typ, size, w):
    if typ == "box":
        h = size[2] if len(size) > 2 else 0.0
        return (w[2] - h, w[2] + h)
    if typ == "cylinder":
        h = size[1] if len(size) > 1 else 0.0
        return (w[2] - h, w[2] + h)
    if typ == "sphere":
        r = size[0] if size else 0.0
        return (w[2] - r, w[2] + r)
    return (w[2], w[2])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xml", default="motion_control/module/sim_module/model/mjcf/environment/lab_env.xml")
    ap.add_argument("--out", default="doc/lab_env_map.png")
    ap.add_argument("--z-min", type=float, default=0.15)
    ap.add_argument("--z-max", type=float, default=1.40)
    args = ap.parse_args()

    wb = ET.parse(args.xml).getroot().find("worldbody")
    geoms = geom_footprints(wb)

    fig, ax = plt.subplots(figsize=(17.6, 9.4))

    # 地板 + 房间外框
    ax.add_patch(Rectangle((-10, -5), 20, 10, fc="#eceae4", ec="none", zorder=0))
    ax.add_patch(Rectangle((-10, -5), 20, 10, fc="none", ec="#2b2f36", lw=2.5, zorder=5))

    dyn_geoms = []
    n_static = n_excl = 0
    for name, typ, mat, rgba, size, wp in geoms:
        if name.startswith("dyn_"):
            dyn_geoms.append((name, typ, rgba, size, wp))
            continue
        if name in ("floor", "ceiling") or "skybox" in name:
            n_excl += 1
            continue
        zl, zh = z_span(name, typ, size, wp)
        if zh <= args.z_min or zl >= args.z_max:      # LED 灯条等
            n_excl += 1
            continue
        if typ not in ("box", "cylinder", "sphere") or not size:
            n_excl += 1
            continue

        color, alpha = MAT_COLOR.get(mat, ("#888888", 1.0))
        lw = 1.8 if mat == "wall" else 0.6
        ec = "#22262c" if mat == "wall" else color

        if typ == "box":
            sx, sy = size[0], size[1]
            ax.add_patch(Rectangle((wp[0] - sx, wp[1] - sy), 2 * sx, 2 * sy,
                                   fc=color, ec=ec, lw=lw, alpha=alpha, zorder=3))
        else:  # cylinder / sphere → 圆
            r = size[0]
            ax.add_patch(Circle((wp[0], wp[1]), r, fc=color, ec=ec,
                                lw=lw, alpha=alpha, zorder=3))
        n_static += 1

    # 玻璃 (半透明叠加描边, 放最上层静态)
    for name, typ, mat, rgba, size, wp in geoms:
        if not name.startswith("glass") or name.endswith(("frameL", "frameR")) or typ != "box":
            continue
        sx, sy = size[0], size[1]
        ax.add_patch(Rectangle((wp[0] - sx, wp[1] - sy), 2 * sx, 2 * sy,
                               fc="#7fd0ea", ec="#2b8fb8", lw=1.4, alpha=0.45, zorder=4))

    # 动态障碍 (虚线边框 + 斜线填充)
    for name, typ, rgba, size, wp in dyn_geoms:
        key = "_".join(name.split("_")[:2])
        c = DYN_COLOR.get(key, "#e04a1f")
        r = size[0]
        if typ == "box":
            sx, sy = size[0], size[1]
            ax.add_patch(Rectangle((wp[0] - sx, wp[1] - sy), 2 * sx, 2 * sy,
                                   fc=c, ec="k", lw=1.2, alpha=0.75,
                                   hatch="///", zorder=4))
        else:
            ax.add_patch(Circle((wp[0], wp[1]), r, fc=c, ec="k", lw=1.2,
                                alpha=0.75, hatch="///", zorder=4))

    # ── 通道标注 ──────────────────────────────────────────────
    def gap_annot(x, y_lo, y_hi, label, color, ok=True):
        ax.annotate("", (x, y_lo), (x, y_hi),
                    arrowprops=dict(arrowstyle="<->", color=color, lw=2.2))
        ax.text(x + 0.18, (y_lo + y_hi) / 2, label, fontsize=11, color=color,
                va="center", weight="bold")
        if not ok:
            ax.text(x + 0.18, (y_lo + y_hi) / 2 - 0.55, "[不可通行] 0.35m < 机身 0.45m",
                    fontsize=10, color=color, va="center")

    gap_annot(1.55, -3.5, -2.5, "通道A 1.00m [可通行]", "#1a7a1a", ok=True)
    gap_annot(1.55, 3.025, 3.375, "通道B 0.35m", "#b02020", ok=False)
    ax.annotate("", (4.7, -1.8), (4.7, 1.0),
                arrowprops=dict(arrowstyle="<->", color="#1a6f9a", lw=2.2))
    ax.text(4.45, -0.4, "玻璃开口 2.8m", fontsize=11, color="#1a6f9a",
            rotation=90, va="center", ha="right", weight="bold")
    ax.text(2.0, 5.45, "中央隔断墙 X=2 (高2.6m)", fontsize=10, color="#3a3f47", ha="center")
    ax.text(5.0, -5.45, "玻璃隔断 X=5", fontsize=10, color="#1a6f9a", ha="center")

    # ── 机器人起点 + 足迹 (空心圆, 避免与场景终点★ 重叠遮挡, 如 E→(0,0)) ──
    ax.add_patch(Circle((0, 0), 0.225, fc="none", ec="k", lw=1.6, ls=":", zorder=5))
    ax.plot(0, 0, "o", ms=11, mfc="white", mec="k", mew=2, zorder=5)
    ax.text(0.38, -0.62, "机器人初始位 (0,0)\n机身宽 0.45m", fontsize=9.5, zorder=7)

    # ── 场景链: 起点○ → 终点★, 直线仅示链序 ──────────────────
    # 注意: A 与 F 路线完全相同 (F=复跑A), E 终点=初始位 — 重叠场景需可分辨
    keyed = defaultdict(list)
    for name, s, g, c in SCENARIOS:
        keyed[(s, g)].append((name, c))

    for (s, g), items in keyed.items():
        # 箭头: 首个场景实线, 重复场景虚线叠加
        for k, (name, c) in enumerate(items):
            ls = "-" if k == 0 else (0, (4, 3))
            ax.annotate("", g, s,
                        arrowprops=dict(arrowstyle="-|>", color=c, lw=2.0 if k == 0 else 1.5,
                                        alpha=0.9 if k == 0 else 0.85,
                                        shrinkA=8, shrinkB=8, linestyle=ls))
        ax.plot(*s, "o", color=items[0][1], ms=9, mec="k", mew=0.7, zorder=6)
        # 终点星: 单场景一颗; 重复路线 (A/F) 双色同心星 — 外圈后跑的, 内圈先跑的
        if len(items) == 1:
            ax.plot(*g, "*", color=items[0][1], ms=17, mec="k", mew=0.6, zorder=6)
        else:
            ax.plot(*g, "*", color=items[-1][1], ms=23, mec="k", mew=0.6, zorder=6)
            ax.plot(*g, "*", color=items[0][1], ms=12, mec="k", mew=0.4, zorder=7)
        # 标签: 重复场景垂直堆叠
        dx, dy = 0.42, 0.32
        if g[0] > 4:
            dx = -0.42
        for k, (name, c) in enumerate(items):
            suffix = " (复跑)" if k > 0 else ""
            ax.text(g[0] + dx, g[1] + dy - 0.62 * k,
                    f"{name[0]}{suffix}→({g[0]:g},{g[1]:g})", fontsize=10.5,
                    color=c, ha="center", weight="bold", zorder=7,
                    bbox=dict(fc="white", ec=c, alpha=0.85, boxstyle="round,pad=0.22"))
    # D 特注: 目标点位于玻璃墙内 (不可达鲁棒性测试)
    ax.text(5.0, 3.2 + 0.85, "D目标在玻璃墙内 · 不可达", fontsize=9, color="#9467bd",
            ha="center", zorder=7,
            bbox=dict(fc="white", ec="#9467bd", alpha=0.8, boxstyle="round,pad=0.2"))

    # ── 图例 / 轴 / 标题 ─────────────────────────────────────
    handles = [
        Rectangle((0, 0), 1, 1, fc="#3a3f47", ec="#22262c", label="墙体/隔断/柱 (conaffinity=7)"),
        Rectangle((0, 0), 1, 1, fc="#7fd0ea", ec="#2b8fb8", alpha=0.5, label="玻璃隔断 (半透明)"),
        Rectangle((0, 0), 1, 1, fc="#b08050", ec="#b08050", label="家具/台面 (wood/metal/chair)"),
        Rectangle((0, 0), 1, 1, fc="#c8a066", ec="#c8a066", label="纸箱 (carton)"),
        Rectangle((0, 0), 1, 1, fc="#f2c400", ec="#f2c400", label="通道警示柱"),
        Line2D([], [], color="#e04a1f", marker="o", ls="none", mec="k",
               label="dyn_* 动态障碍 (CI 中未驱动 mocap, 实为静态)"),
        Line2D([], [], color="k", marker="*", ls=":", mec="k", label="场景起点○ / 终点★ (A→F 串联)"),
    ]
    ax.legend(handles=handles, loc="lower left", fontsize=9.5, framealpha=0.95,
              bbox_to_anchor=(-0.005, -0.005))

    # 比例尺 2m
    ax.plot([-9.6, -7.6], [-4.6, -4.6], color="k", lw=3, solid_capstyle="butt")
    ax.text(-8.6, -4.45, "2 m", fontsize=10, ha="center")

    ax.set_xlim(-10.8, 10.8)
    ax.set_ylim(-6.2, 6.0)
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("CI 全局唯一地图 lab_env.xml — 200㎡ 实验室 20m×10m | 6 场景 A–F 串联 (nav_test_runner.py SCENARIOS)",
                 fontsize=13, weight="bold")
    ax.grid(True, ls="--", lw=0.4, alpha=0.35)
    ax.tick_params(labelsize=9)
    fig.text(0.99, 0.01, f"静态障碍 {n_static} 个 (z∈[0.15,1.40]m) | 排除 {n_excl} 个 | A/F 路线相同 (F=复跑A): 双色同心星, 内圈A外圈F, F箭头为虚线 | 场景箭头仅示目标链序, 非实际规划路径",
             fontsize=8.5, ha="right", color="#555555")

    fig.tight_layout()
    fig.savefig(args.out, dpi=150, bbox_inches="tight", facecolor="white")
    print(f"static geoms drawn: {n_static}, excluded: {n_excl}, dynamic: {len(dyn_geoms)}")
    print(f"saved: {args.out}")


if __name__ == "__main__":
    main()
