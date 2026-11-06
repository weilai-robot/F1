#!/usr/bin/env python3
# ============================================================
# bag_analyze.py — rosbag2 离线导航指标提取（v0，零 ROS 依赖）
#
# 直接读 .db3（sqlite3）+ 手解 CDR，不需要 ros2 环境——
# WSL（desk 开发）/ 81（sim bag）/ minipc（real bag）三处均可跑。
#
# v0 指标（用户最小集，2026-11-05 定）：
#   时长 / 路径长度 / 平均+最大速度 / 终点差距(xy+yaw) / 成功状态
#   路径长度比（有 /plan 时）/ cmd_vel 统计（含 wz 饱和占比，尾段旋转归因用）
#   /stand_mode /walk_mode 事件时间线（P0 验收用）
#
# 用法:
#   python3 bag_analyze.py BAG [BAG ...] [--goal X Y YAW]
#       [--xy-tol 0.5] [--yaw-tol 0.5] [--wz-sat 0.35]
#       [--csv out.csv] [--json out.json] [--dump-timeline out.csv]
#   BAG = .db3 文件，或 rosbag2 目录（含 .db3，支持多分片）
# ============================================================
import argparse
import glob
import json
import math
import os
import sqlite3
import struct
import sys


class CDR:
    """最小 CDR（XCDR1 LE）读取器。对齐基准 = 封装头(4B)之后。"""

    __slots__ = ("d", "pos")

    def __init__(self, data):
        if data[:2] not in (b"\x00\x01", b"\x00\x03"):
            raise ValueError("非 CDR 封装头: %r" % (data[:4],))
        self.d = data
        self.pos = 4

    def _align(self, n):
        r = (self.pos - 4) % n
        if r:
            self.pos += n - r

    def u32(self):
        self._align(4)
        v = struct.unpack_from("<I", self.d, self.pos)[0]
        self.pos += 4
        return v

    def i8(self):
        v = struct.unpack_from("<b", self.d, self.pos)[0]
        self.pos += 1
        return v

    def f64(self):
        self._align(8)
        v = struct.unpack_from("<d", self.d, self.pos)[0]
        self.pos += 8
        return v

    def s(self):
        n = self.u32()
        b = self.d[self.pos:self.pos + n]
        self.pos += n
        return b.split(b"\x00")[0].decode("utf-8", "replace")

    def skip_doubles(self, n):
        self._align(8)
        self.pos += 8 * n

    def skip_bytes(self, n):
        self.pos += n


def yaw_from_q(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# ---------------- 消息解码（按需） ----------------

def dec_tf(data):
    c = CDR(data)
    out = []
    for _ in range(c.u32()):
        c.u32(); c.u32()  # stamp（用 bag 录制时钟，不用 header 时钟——两者可能差一整个会话）
        parent = c.s()
        child = c.s()
        tx, ty, tz = c.f64(), c.f64(), c.f64()
        qx, qy, qz, qw = c.f64(), c.f64(), c.f64(), c.f64()
        out.append((parent, child, tx, ty, yaw_from_q(qx, qy, qz, qw)))
    return out


def dec_odom(data):
    c = CDR(data)
    c.u32(); c.u32()  # stamp
    frame = c.s()
    c.s()  # child_frame_id
    px, py = c.f64(), c.f64()
    c.f64()  # pz
    qx, qy, qz, qw = c.f64(), c.f64(), c.f64(), c.f64()
    c.skip_doubles(36)  # pose covariance
    lvx, lvy = c.f64(), c.f64()
    c.f64()
    avz = c.f64()
    c.f64(); c.f64()
    return (frame, px, py, yaw_from_q(qx, qy, qz, qw), lvx, lvy, avz)


def dec_twist(data, stamped):
    c = CDR(data)
    if stamped:
        c.u32(); c.u32()  # stamp
        c.s()             # frame_id
    vx, vy, vz = c.f64(), c.f64(), c.f64()
    wx, wy, wz = c.f64(), c.f64(), c.f64()
    return (vx, vy, wz)


def dec_path(data):
    c = CDR(data)
    c.u32(); c.u32()  # stamp
    c.s()             # frame_id
    pts = []
    for _ in range(c.u32()):
        c.u32(); c.u32()  # pose stamp
        c.s()             # pose frame_id
        pts.append((c.f64(), c.f64()))
        c.f64()  # z
        c.skip_doubles(4)  # orientation
    return pts


def dec_status(data):
    c = CDR(data)
    last = None
    for _ in range(c.u32()):
        c.skip_bytes(16)      # goal_id uuid（align 1）
        c.u32(); c.u32()      # stamp
        last = c.i8()
    return last


STATUS_NAME = {0: "UNKNOWN", 1: "ACCEPTED", 2: "EXECUTING", 3: "CANCEL_REQ",
               4: "SUCCEEDED", 5: "ABORTED", 6: "CANCELED"}


def dec_std(data, typ):
    """std_msgs 简单类型（mode 事件载荷），失败返回 None。"""
    try:
        c = CDR(data)
        if "Bool" in typ:
            return bool(c.d[c.pos])
        if "Int32" in typ or "UInt32" in typ:
            return c.u32()
        if "String" in typ:
            return c.s()
        return None  # Empty 等：只有事件时刻有意义
    except Exception:
        return None


# ---------------- bag 读取 ----------------

WANTED = ("/tf", "/tf_static", "/cmd_vel", "/cmd_vel_limiter", "/plan",
          "/navigate_to_pose/_action/status", "/stand_mode", "/walk_mode")


def is_wanted(name, typ):
    if name in WANTED:
        return True
    if typ.endswith("msg/Odometry") or typ.endswith("/Odometry"):
        return True
    if name.endswith("/_action/status"):
        return True
    return False


def read_bag(path):
    dbs = sorted(glob.glob(os.path.join(path, "*.db3"))) if os.path.isdir(path) else [path]
    if not dbs:
        sys.exit(f"[bag_analyze] {path} 里没有 .db3")
    topics, msgs = {}, {}
    for db in dbs:
        con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
        tid_map = {}
        for tid, name, typ in con.execute("select id,name,type from topics"):
            tid_map[tid] = name
            if is_wanted(name, typ) and name not in msgs:
                msgs[name] = (typ, [])
        q = ",".join(str(t) for t in tid_map if tid_map[t] in msgs) or "NULL"
        for tid, ts, blob in con.execute(
                f"select topic_id,timestamp,data from messages where topic_id in ({q}) order by timestamp"):
            msgs[tid_map[tid]][1].append((ts * 1e-9, blob))  # 统一用录制时钟（秒）
        con.close()
    for name in msgs:
        msgs[name][1].sort(key=lambda x: x[0])
    return msgs


# ---------------- 位姿时间线 ----------------

def build_timeline(msgs):
    """优先 /tf 组合 map→base；无 map→odom 则退 /odom 或 odom→base（odom 系）。"""
    tfs = [(t, dec_tf(b)) for t, b in msgs.get("/tf", ([], []))[1]]
    mo = [(t, x, y, yaw) for t, trs in tfs
          for p, c, x, y, yaw in trs if p == "map" and c == "odom"]
    ob = [(t, x, y, yaw) for t, trs in tfs
          for p, c, x, y, yaw in trs
          if p == "odom" and c in ("base_footprint", "base_link")]

    if mo and ob:
        out, j = [], 0
        for t, bx, by, byaw in ob:
            while j + 1 < len(mo) and abs(mo[j + 1][0] - t) <= abs(mo[j][0] - t):
                j += 1
            if abs(mo[j][0] - t) > 0.5:
                continue
            _, mx, my, myaw = mo[j]
            cy, sy = math.cos(myaw), math.sin(myaw)
            out.append((t, mx + cy * bx - sy * by, my + sy * bx + cy * by,
                        wrap(myaw + byaw)))
        if out:
            return out, "map"

    for name in ("/odom", "/Odometry"):
        if name in msgs and msgs[name][1]:
            return [(t, p[1], p[2], p[3]) for t, p in
                    ((t, dec_odom(b)) for t, b in msgs[name][1])], "odom"
    if ob:
        return [(t, x, y, yaw) for t, x, y, yaw in ob], "odom(tf)"
    # fastlio 裸包：camera_init→body 也是一条可用轨迹（建图/录包验证用）
    fb = [(t, x, y, yaw) for t, trs in tfs
          for p, c, x, y, yaw in trs if p == "camera_init" and c == "body"]
    if fb:
        return fb, "camera_init(fastlio)"
    return [], None


def path_len(tl):
    tot = 0.0
    for a, b in zip(tl, tl[1:]):
        dt = min(max(b[0] - a[0], 1e-3), 0.5)
        d = math.hypot(b[1] - a[1], b[2] - a[2])
        if d / dt < 5.0:  # 跳变（重定位/漂移）不计入路径
            tot += d
    return tot


def smooth_xy(tl, win_s):
    """x/y 中心滑动平均（时间窗 win_s 秒）。

    足式里程计在步频附近有侧摆噪声（10Hz 采样下路径虚高 ~1.5×，
    sim GT 对照实测：15.02m→9.41m @0.4s，GT=9.56m），积分路径前先滤。
    yaw 不滤（终点朝向要原始值）。win_s<=0 原样返回。
    """
    if win_s <= 0 or len(tl) < 3:
        return tl
    ts = [e[0] for e in tl]
    half = win_s / 2.0
    out = []
    for i, (t, x, y, yaw) in enumerate(tl):
        j = i
        while j > 0 and t - ts[j] < half:
            j -= 1
        k = i
        while k + 1 < len(ts) and ts[k] - t < half:
            k += 1
        seg = tl[j:k + 1]
        n = len(seg)
        out.append((t, sum(e[1] for e in seg) / n,
                    sum(e[2] for e in seg) / n, yaw))
    return out


# ---------------- 主分析 ----------------

def analyze(bag, args):
    msgs = read_bag(bag)
    r = {"bag": bag, "frame": None, "notes": []}
    tl, frame = build_timeline(msgs)
    r["frame"] = frame
    r["_tl"] = tl
    if not tl:
        r["notes"].append("无可用位姿时间线（/tf 或 /odom）")
        return r

    t0 = tl[0][0]
    r["_t0"] = t0
    dur = tl[-1][0] - t0
    s_tl = smooth_xy(tl, getattr(args, "smooth_s", 0.4))
    r["smooth_s"] = getattr(args, "smooth_s", 0.4)
    plen = path_len(s_tl)
    speeds = []
    for a, b in zip(s_tl, s_tl[1:]):
        dt = min(max(b[0] - a[0], 1e-3), 0.5)
        d = math.hypot(b[1] - a[1], b[2] - a[2])
        if d / dt < 5.0:
            speeds.append(d / dt)
    r.update(dur_s=round(dur, 1), path_m=round(plen, 2),
             v_avg=round(plen / dur, 3) if dur > 0 else None,
             v_max=round(max(speeds), 3) if speeds else None)

    # 终点差距
    if args.goal:
        gx, gy, gyaw = args.goal
        _, ex, ey, eyaw = tl[-1]
        r["goal_err_xy"] = round(math.hypot(gx - ex, gy - ey), 3)
        r["goal_err_yaw"] = round(abs(wrap(gyaw - eyaw)), 3)
        ok = r["goal_err_xy"] <= args.xy_tol and r["goal_err_yaw"] <= args.yaw_tol
        r["success_heuristic"] = bool(ok) if dur <= args.timeout else False
        if dur > args.timeout:
            r["notes"].append(f"超时（{dur:.0f}s > {args.timeout}s）")

    # action status（真值优先）
    for name, (typ, arr) in msgs.items():
        if name.endswith("/_action/status") and arr:
            st = dec_status(arr[-1][1])
            r["action_status"] = STATUS_NAME.get(st, st)
            r["action_topic"] = name
            if arr:
                r["status_t_rel_s"] = round(arr[-1][0] - t0, 1)

    # 路径长度比
    if "/plan" in msgs and msgs["/plan"][1]:
        pts = dec_path(msgs["/plan"][1][0][1])
        pl = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
        r["plan_m"] = round(pl, 2)
        if pl > 0.1:
            r["path_ratio"] = round(plen / pl, 2)

    # cmd_vel 统计
    cv = None
    for name in ("/cmd_vel_limiter", "/cmd_vel"):
        if name in msgs and msgs[name][1]:
            stamped = "TwistStamped" in msgs[name][0]
            cv = [dec_twist(b, stamped) for _, b in msgs[name][1]]
            r["cmd_vel_topic"] = name
            break
    if cv:
        wz = [abs(w) for _, _, w in cv]
        r["cmd_n"] = len(cv)
        r["v_max_cmd"] = round(max(math.hypot(vx, vy) for vx, vy, _ in cv), 3)
        r["wz_max_cmd"] = round(max(wz), 3)
        r["wz_sat_frac"] = round(sum(w > args.wz_sat for w in wz) / len(wz), 3)

    # mode 事件（P0：stand 先于 cmd_vel 归零）
    for name in ("/stand_mode", "/walk_mode"):
        if name in msgs and msgs[name][1]:
            ev = []
            for t, b in msgs[name][1]:
                v = dec_std(b, msgs[name][0])
                ev.append((round(t - t0, 1), v if v is not None else ""))
            r[name.strip("/").replace("/", "_") + "_events"] = ev[:10]

    return r


def main():
    ap = argparse.ArgumentParser(description="rosbag2 离线导航指标提取（零 ROS 依赖）")
    ap.add_argument("bags", nargs="+")
    ap.add_argument("--goal", nargs=3, type=float, metavar=("X", "Y", "YAW"))
    ap.add_argument("--xy-tol", type=float, default=0.5)
    ap.add_argument("--yaw-tol", type=float, default=0.5)
    ap.add_argument("--timeout", type=float, default=120.0)
    ap.add_argument("--wz-sat", type=float, default=0.35)
    ap.add_argument("--smooth-s", type=float, default=0.4,
                    help="路径低通时间窗（秒，步频侧摆滤除）；0=关闭")
    ap.add_argument("--csv")
    ap.add_argument("--json")
    ap.add_argument("--dump-timeline")
    args = ap.parse_args()
    results = [analyze(b, args) for b in args.bags]

    if args.dump_timeline:  # PlotJuggler/复盘用：t_rel_s,x,y,yaw
        os.makedirs(args.dump_timeline, exist_ok=True)
        for r in results:
            base = os.path.basename(r["bag"].rstrip("/"))
            with open(os.path.join(args.dump_timeline, base + "_timeline.csv"), "w") as f:
                f.write("t_rel_s,x,y,yaw\n")
                for t, x, y, yaw in r.get("_tl", []):
                    f.write(f"{t - r['_t0']:.3f},{x:.4f},{y:.4f},{yaw:.4f}\n")
        print(f"时间线已写: {args.dump_timeline}/*_timeline.csv")

    for r in results:
        print("=" * 60)
        print(r["bag"], f"[{r['frame']}]")
        for k, v in r.items():
            if k not in ("bag", "frame", "notes") and not k.startswith("_"):
                print(f"  {k:18} {v}")
        for n in r.get("notes", []):
            print(f"  ! {n}")
    if args.csv or args.json:
        keys = sorted({k for r in results for k in r if not k.startswith("_")})
        if args.csv:
            with open(args.csv, "w") as f:
                f.write(",".join(keys) + "\n")
                for r in results:
                    f.write(",".join(str(r.get(k, "")) for k in keys) + "\n")
        if args.json:
            with open(args.json, "w") as f:
                json.dump(results, f, ensure_ascii=False, indent=1, default=str)
        print(f"\n已写: {args.csv or ''}{args.json or ''}")


if __name__ == "__main__":
    main()
