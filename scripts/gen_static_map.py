#!/usr/bin/env python3
"""
gen_static_map.py — 从 MuJoCo 场景 XML 真值生成 Nav2 静态占据地图

背景: 仓库中的 mujoco_lab.pgm 是旧 FastLIO 扫描, 与 lab_env.xml 当前布局
不一致 (南门 y=-3 实宽 1.0m 但旧图仅 0.2m 缝; 东房 lab1 工作台 (6.5,-1.2)
旧图缺失)。过时地图导致: 全局规划误绕路 + 门禁 A* 参考失真。

规则:
  - 遍历 worldbody 下全部 geom (含嵌套 body 的 pos 累加)
  - 排除: dyn_* 动态障碍 (mocap 可移动, 由 live voxel 层负责)、
          floor/ceiling/skybox、z 波段与 [0.15m, 1.4m] 无交集的 (led 灯等)
  - box: 足迹 = pos ± size(xy); cylinder/sphere: 圆 r=size[0]
  - 栅格 0.05m, 帧对齐旧图 (401×204, origin=(-9.95,-5.1)) — 仅地图内容更新
  - PGM: 254=自由, 0=占据, 205=未知(房间外)

用法:
  python3 scripts/gen_static_map.py --xml motion_control/module/sim_module/model/mjcf/environment/lab_env.xml \
      --out navigation/planning/humanoid_sim/maps/mujoco_lab.pgm [--dry-run]
"""

import argparse
import math
import xml.etree.ElementTree as ET


def geom_footprints(root):
    """产出 [(name, kind, params, world_pos)] — 世界系足迹图元"""
    out = []

    def walk(elem, base):
        for g in elem.findall("geom"):
            name = g.get("name", "")
            typ = g.get("type", "")
            size = [float(v) for v in (g.get("size") or "").split() if v]
            pos = [float(v) for v in (g.get("pos") or "0 0 0").split()]
            if len(pos) < 3:
                pos = pos + [0.0] * (3 - len(pos))
            w = (base[0] + pos[0], base[1] + pos[1], base[2] + pos[2])
            out.append((name, typ, size, w))
        for b in elem.findall("body"):
            p = [float(v) for v in (b.get("pos") or "0 0 0").split()]
            p = p + [0.0] * (3 - len(p))
            nb = (base[0] + p[0], base[1] + p[1], base[2] + p[2])
            walk(b, nb)

    walk(root, (0.0, 0.0, 0.0))
    return out


def z_span(name, typ, size, w):
    """geom 的 z 区间 [z_lo, z_hi]"""
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
    ap.add_argument("--xml", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--res", type=float, default=0.05)
    ap.add_argument("--origin", default="-9.95,-5.1")
    ap.add_argument("--z-min", type=float, default=0.15, help="低于此高度的障碍不挡机器人")
    ap.add_argument("--z-max", type=float, default=1.40, help="高于此高度(lidar 上界)不标")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    ox, oy = [float(v) for v in args.origin.split(",")]
    root = ET.parse(args.xml).getroot()
    wb = root.find("worldbody")
    geoms = geom_footprints(wb if wb is not None else root)

    # 帧大小: 对齐旧图 401×204 (x∈[-9.95,10.1], y∈[-5.1,5.1])
    w, h = 401, 204

    include, exclude = [], []
    for name, typ, size, wp in geoms:
        zl, zh = z_span(name, typ, size, wp)
        if name.startswith("dyn_"):
            exclude.append((name, "dynamic")); continue
        if name in ("floor", "ceiling") or "skybox" in name:
            exclude.append((name, "struct")); continue
        if zh <= args.z_min or zl >= args.z_max:
            exclude.append((name, f"z[{zl:.2f},{zh:.2f}]")); continue
        if typ not in ("box", "cylinder", "sphere") or not size:
            exclude.append((name, f"type {typ}")); continue
        include.append((name, typ, size, wp))

    # 栅格化
    occ = [[False] * w for _ in range(h)]

    def mark_cell(gx, gy):
        if 0 <= gx < w and 0 <= gy < h:
            occ[gy][gx] = True

    for name, typ, size, wp in include:
        if typ == "box":
            sx, sy = size[0], size[1]
            for gy in range(h):
                cy = oy + (gy + 0.5) * args.res
                if not (wp[1] - sy <= cy <= wp[1] + sy):
                    continue
                gx_lo = int(math.floor((wp[0] - sx - ox) / args.res))
                gx_hi = int(math.ceil((wp[0] + sx - ox) / args.res))
                for gx in range(gx_lo, gx_hi + 1):
                    mark_cell(gx, gy)
        else:  # cylinder / sphere → 圆
            r = size[0]
            for gy in range(h):
                cy = oy + (gy + 0.5) * args.res
                dy = cy - wp[1]
                if abs(dy) > r:
                    continue
                dx = math.sqrt(max(r * r - dy * dy, 0.0))
                gx_lo = int(math.floor((wp[0] - dx - ox) / args.res))
                gx_hi = int(math.ceil((wp[0] + dx - ox) / args.res))
                for gx in range(gx_lo, gx_hi + 1):
                    mark_cell(gx, gy)

    # 房间外 → 未知 (房间 x∈[-10.05,10.05], y∈[-5.05,5.05], 留 0.1 墙厚)
    def in_room(gx, gy):
        cx = ox + (gx + 0.5) * args.res
        cy = oy + (gy + 0.5) * args.res
        return -9.9 <= cx <= 9.9 and -4.9 <= cy <= 4.9

    # 写 PGM (P5): 行序自上而下 (+y 在上)
    rows = []
    occ_n = free_n = unk_n = 0
    for gy in range(h - 1, -1, -1):
        row = bytearray(w)
        for gx in range(w):
            if not in_room(gx, gy):
                row[gx] = 205; unk_n += 1
            elif occ[gy][gx]:
                row[gx] = 0; occ_n += 1
            else:
                row[gx] = 254; free_n += 1
        rows.append(bytes(row))

    total = w * h
    print(f"geoms: include={len(include)} exclude={len(exclude)}")
    for nm, why in exclude:
        print(f"  excl: {nm:<24} {why}")
    print(f"cells: occ={occ_n} ({100*occ_n/total:.1f}%) free={free_n} unknown={unk_n}")

    if args.dry_run:
        return
    with open(args.out, "wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode())
        for r in rows:
            f.write(r)
    with open(args.out.replace(".pgm", ".yaml"), "w") as f:
        f.write(f"image: {args.out.split('/')[-1]}\n"
                f"mode: trinary\nresolution: {args.res}\n"
                f"origin: [{ox}, {oy}, 0]\nnegate: 0\n"
                f"occupied_thresh: 0.65\nfree_thresh: 0.25\n")
    print(f"written: {args.out} (+yaml)")


if __name__ == "__main__":
    main()
