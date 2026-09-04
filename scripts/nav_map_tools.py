#!/usr/bin/env python3
"""
nav_map_tools.py — 测试地图工具: 栅格加载 + 间隙场 + A* 最优路径

用于严格门禁 (nav_strict_gate.py):
  - 从 trials 的实际起点/终点计算几何最优路径长度
  - 路径效率 = 最优路径 / 实际位移 (对地图结构公平)
  - 完成时间预算 = 最优路径 / 目标速度 + 固定余量

用法 (CLI 自检):
  python3 scripts/nav_map_tools.py --map navigation/planning/humanoid_sim/maps/mujoco_lab.yaml \
      --pairs 0,0,5,0 0,0,5,-3 5,-3,5,3.2
"""

import argparse
import heapq
import math
import struct


# ── PGM (P5/P2) 读取 (无 PIL 依赖, runner 环境安全) ──────────
def load_pgm(path: str):
    with open(path, "rb") as f:
        data = f.read()
    # 解析 header: magic, width, height, maxval (跳过注释)
    tokens = []
    i = 0
    while len(tokens) < 4:
        # 跳过空白
        while i < len(data) and data[i:i+1].isspace():
            i += 1
        if data[i:i+1] == b"#":
            while i < len(data) and data[i:i+1] != b"\n":
                i += 1
            continue
        j = i
        while j < len(data) and not data[j:j+1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    magic = tokens[0]
    w, h, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic == b"P5":
        i += 1  # 单字节分隔
        pix = data[i:i + w * h]
        if maxval > 255:  # 16-bit big endian
            vals = struct.unpack(f">{w*h}H", data[i:i + 2*w*h])
        else:
            vals = list(pix)
    elif magic == b"P2":
        vals = [int(x) for x in data[i:].split()[:w*h]]
    else:
        raise ValueError(f"unsupported pgm magic {magic}")
    return w, h, vals


class NavMap:
    """占据栅格 + 间隙场 (chamfer 距离变换) + 带间隙约束的 A*"""

    def __init__(self, yaml_path: str, robot_radius: float = 0.30,
                 occupied_thresh: float = 0.65):
        import os
        base = os.path.dirname(os.path.abspath(yaml_path))
        cfg = {}
        with open(yaml_path) as f:
            for line in f:
                if ":" in line:
                    k, v = line.split(":", 1)
                    cfg[k.strip()] = v.strip()
        self.resolution = float(cfg.get("resolution", 0.05))
        ox, oy = cfg.get("origin", "[-9.95, -5.1, 0]").strip("[] ").split(",")[:2]
        self.origin = (float(ox), float(oy))
        self.negate = int(cfg.get("negate", 0))
        self.occ_thresh = float(cfg.get("occupied_thresh", occupied_thresh))
        w, h, vals = load_pgm(os.path.join(base, cfg.get("image", "map.pgm")))
        self.w, self.h = w, h
        # PGM 约定 row0=顶部=+y_max; 按行翻转为 y_min 起始的行主序 (w2g 直接索引)
        # 注意: 不能用 vals[::-1] (那会把每行内部也反转 = 180°旋转)
        vals = [vals[r * w:(r + 1) * w] for r in range(h - 1, -1, -1)]
        vals = [v for row in vals for v in row]
        # pgm 值 → 占据概率: p = (255 - v)/255 (negate=0)
        occ = [0.0] * (w * h)
        for idx, v in enumerate(vals):
            p = v / 255.0 if self.negate else (255 - v) / 255.0
            occ[idx] = p
        self.free = [p < self.occ_thresh for p in occ]
        self.robot_radius = robot_radius
        self._clearance = None

    # ── 世界坐标 ↔ 栅格 ────────────────────────────────────
    def w2g(self, x: float, y: float):
        gx = int((x - self.origin[0]) / self.resolution)
        gy = int((y - self.origin[1]) / self.resolution)
        return (min(max(gx, 0), self.w - 1), min(max(gy, 0), self.h - 1))

    # ── 间隙场: 每个自由格到最近占据格的距离 (米), chamfer 2-pass ──
    def clearance(self):
        if self._clearance is not None:
            return self._clearance
        INF = float("inf")
        w, h, r = self.w, self.h, self.resolution
        d = [INF] * (w * h)
        for idx in range(w * h):
            if not self.free[idx]:
                d[idx] = 0.0
        # 前向 + 后向 chamfer (3-4 mask)
        D1, D2 = 1.0, math.sqrt(2.0)

        def relax(gy_iter, gx_iter, neigh):
            for gy in gy_iter:
                for gx in gx_iter:
                    i = gy * w + gx
                    if d[i] == 0.0:
                        continue
                    best = d[i]
                    for dy, dx, c in neigh:
                        ny, nx = gy + dy, gx + dx
                        if 0 <= ny < h and 0 <= nx < w:
                            v = d[ny * w + nx] + c
                            if v < best:
                                best = v
                    d[i] = best

        # 前向: 自上而下、自左而右, 用 (上, 左, 左上, 右上)
        relax(range(1, h - 1), range(1, w - 1),
              ((-1, 0, D1), (0, -1, D1), (-1, -1, D2), (-1, 1, D2)))
        # 后向: 自下而上、自右而左, 用 (下, 右, 右下, 左下)
        relax(range(h - 2, 0, -1), range(w - 2, 0, -1),
              ((1, 0, D1), (0, 1, D1), (1, 1, D2), (1, -1, D2)))
        self._clearance = [x * r for x in d]
        return self._clearance

    # ── A*: 只走 clearance ≥ robot_radius 的格子, 8-连通 ─────
    def optimal_path_length(self, x0, y0, x1, y1) -> float:
        cl = self.clearance()
        w, h = self.w, self.h
        start, goal = self.w2g(x0, y0), self.w2g(x1, y1)

        def feasible(g):
            return (0 <= g[0] < w and 0 <= g[1] < h
                    and cl[g[1] * w + g[0]] >= self.robot_radius)

        # 起终点若不可行, 放宽到 ≥ 半径 (避免门边角数值问题导致 False)
        if not feasible(start):
            return float("nan")
        if not feasible(goal):
            return float("nan")

        r = self.resolution
        SQ2 = math.sqrt(2)
        INF2 = float("inf")
        open_h = []
        g_cost = {start: 0.0}
        came = {}
        heuristic = lambda a, b: math.hypot(a[0]-b[0], a[1]-b[1]) * r
        heapq.heappush(open_h, (heuristic(start, goal), 0.0, start))
        while open_h:
            _, g, cur = heapq.heappop(open_h)
            if cur == goal:
                # 回溯路径长度
                length = 0.0
                node = cur
                while node in came:
                    prev = came[node]
                    length += math.hypot(node[0]-prev[0], node[1]-prev[1]) * r
                    node = prev
                return length
            if g > g_cost.get(cur, INF2):
                continue
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nxt = (cur[0] + dx, cur[1] + dy)
                    if not feasible(nxt):
                        continue
                    step = r * (SQ2 if dx and dy else 1.0)
                    ng = g + step
                    if ng < g_cost.get(nxt, INF2):
                        g_cost[nxt] = ng
                        came[nxt] = cur
                        heapq.heappush(open_h, (ng + heuristic(nxt, goal), ng, nxt))
        return float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True)
    ap.add_argument("--robot-radius", type=float, default=0.30)
    ap.add_argument("--pairs", nargs="*", default=[],
                    help="x0,y0,x1,y1 列表 (逗号分隔)")
    args = ap.parse_args()
    nm = NavMap(args.map, robot_radius=args.robot_radius)
    print(f"map {args.map}: {nm.w}x{nm.h} res={nm.resolution} origin={nm.origin}")
    for p in args.pairs:
        x0, y0, x1, y1 = [float(v) for v in p.split(",")]
        L = nm.optimal_path_length(x0, y0, x1, y1)
        straight = math.hypot(x1-x0, y1-y0)
        print(f"({x0},{y0})->({x1},{y1}): optimal={L:.2f}m straight={straight:.2f}m "
              f"straight/optimal={straight/L if L==L and L>0 else float('nan'):.3f}")


if __name__ == "__main__":
    main()
