#!/usr/bin/env python3
"""
nav_debug_plot.py — 导航调试日志后处理: 统一到 map 世界系的可视化与导出

读取 nav_debug_record.sh 产生的 bag, 把三路核心数据统一到 map 系 (世界系):
  - /plan                        (map 系, 全局路径)
  - /nav_debug/mppi_optimal_traj (odom 系 -> 经 map->odom TF 变到 map 系)
  - /cmd_vel_limiter, /cmd_vel   (数值, 叠加到机器人轨迹上/时序图)

输出 (默认 <run_dir>/analysis/):
    map_overlay.png   — map 系叠加图: 全局路径 + MPPI 局部轨迹(按时间着色)
                        + 机器人轨迹 + 限速后指令箭头 + 卡停区间标记 (+代价地图快照底图)
    cmd_vel.png       — /cmd_vel vs /cmd_vel_limiter vs /odom 实测 (位姿差分; FastLIO 不填 twist) 时序
    summary.md        — 自动摘要: 指令归零/卡停区间及 map 坐标、数据质量、排查提示
    data/
      robot_traj_map.csv    t_rel_s,t_unix_s,x_map,y_map,yaw_map,x_odom,y_odom,vx_pose,wz_pose,vx_twist,wz_twist
      cmd_vel.csv           t_rel_s,t_unix_s,vx,wz          (限速前, MPPI 原始输出)
      cmd_vel_limiter.csv   t_rel_s,t_unix_s,vx,wz          (限速后, 下发给 RL)
      global_plan.csv       plan_id,t_rel_s,seq,x_map,y_map
      mppi_traj.csv         traj_id,t_rel_s,seq,x_odom,y_odom,x_map,y_map

用法:
  python3 nav_debug_plot.py reports/nav_debug/<dir>      # 目录或直接给 bag 目录
  python3 nav_debug_plot.py <dir> --no-plots             # 仅 CSV+摘要 (无 matplotlib 时)
  python3 nav_debug_plot.py <dir> --stuck-eps 0.08 --stuck-min-dur 0.8

依赖: numpy + ROS2 (rosbag2_py) 必需; matplotlib 可选 (缺省时跳过 png)。
      运行前先 source ROS2 环境 (scripts/ros2_source.sh)。
"""
import argparse
import csv
import os
import sys
from datetime import datetime

try:
    import numpy as np
except ImportError:
    print("[ERROR] 缺少 numpy, 请先安装: pip3 install numpy", file=sys.stderr)
    sys.exit(1)

# 期望的话题 -> ROS2 消息类型
WANTED_TOPICS = {
    "/plan": "nav_msgs/msg/Path",
    "/nav_debug/mppi_optimal_traj": "nav_msgs/msg/Path",
    "/transformed_global_plan": "nav_msgs/msg/Path",
    "/trajectories": "visualization_msgs/msg/MarkerArray",  # 兜底来源(直接录了原始帧时)
    "/cmd_vel": "geometry_msgs/msg/Twist",
    "/cmd_vel_limiter": "geometry_msgs/msg/Twist",
    "/odom": "nav_msgs/msg/Odometry",
    "/Odometry": "nav_msgs/msg/Odometry",          # FastLIO 原始 (camera_init 系)
    "/odom2map": "nav_msgs/msg/Odometry",          # open3d ICP 矩阵 (map→odom)
    "/localization_3d_confidence": "std_msgs/msg/Float32",  # ICP fitness
    "/tf": "tf2_msgs/msg/TFMessage",
    "/local_costmap/costmap": "nav_msgs/msg/OccupancyGrid",
}

# 小脑状态机模式话题 (std_msgs/Float32, 事件式)
MODE_TOPICS = {
    "/walk_mode": "walk",
    "/stand_mode": "stand",
    "/zero_mode": "zero",
    "/idle_mode": "idle",
    "/keep_mode": "keep",
}
WANTED_TOPICS.update({k: "std_msgs/msg/Float32" for k in MODE_TOPICS})

OPTIMAL_NS = "Optimal Trajectory"  # MPPI 最优轨迹 marker 命名空间

# ═══════════════════════════════════════════════════════════
#  纯函数区 (不依赖 ROS, 便于离线测试)
# ═══════════════════════════════════════════════════════════


def yaw_from_quaternion(x, y, z, w):
    """四元数 -> yaw (rad)。"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return float(np.arctan2(siny_cosp, cosy_cosp))


def lerp_angle(a, b, f):
    """角度最短路径线性插值。"""
    d = (b - a + np.pi) % (2.0 * np.pi) - np.pi
    return a + d * f


class PoseSeries:
    """按时间索引的 2D 位姿序列 (x, y, yaw), 支持线性插值取样。"""

    def __init__(self, ts, xs, ys, yaws):
        order = np.argsort(ts)
        self.ts = np.asarray(ts, dtype=float)[order]
        self.xs = np.asarray(xs, dtype=float)[order]
        self.ys = np.asarray(ys, dtype=float)[order]
        self.yaws = np.asarray(yaws, dtype=float)[order]
        self.n_clamped = 0
        self.n_queries = 0

    def __len__(self):
        return len(self.ts)

    def at(self, t):
        """返回 (x, y, yaw, clamped)。空序列 -> None。

        clamped=True 表示 t 在序列时间范围之外, 用了端点值 (常量外推)。
        """
        if len(self.ts) == 0:
            return None
        self.n_queries += 1
        if t < self.ts[0]:
            self.n_clamped += 1
            return self.xs[0], self.ys[0], self.yaws[0], True
        if t > self.ts[-1]:
            self.n_clamped += 1
            return self.xs[-1], self.ys[-1], self.yaws[-1], True
        i = int(np.searchsorted(self.ts, t, side="right"))
        i = min(max(i, 1), len(self.ts) - 1)
        t0, t1 = self.ts[i - 1], self.ts[i]
        f = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
        x = self.xs[i - 1] + (self.xs[i] - self.xs[i - 1]) * f
        y = self.ys[i - 1] + (self.ys[i] - self.ys[i - 1]) * f
        yaw = lerp_angle(self.yaws[i - 1], self.yaws[i], f)
        return x, y, yaw, False


def transform_points(pts_xy, xo, yo, yaw):
    """把 Nx2 点集按 (x,y,yaw) 做刚体变换。"""
    pts = np.asarray(pts_xy, dtype=float)
    if pts.size == 0:
        return pts.reshape(0, 2)
    c, s = np.cos(yaw), np.sin(yaw)
    rot = np.array([[c, -s], [s, c]])
    return pts @ rot.T + np.array([xo, yo])


def find_segments(mask, ts, min_dur=1.0, max_gap=0.5):
    """把布尔掩码转成连续区间列表 [(t0, t1, i0, i1), ...]。

    ts 需单调递增 (秒)。max_gap: 样本间允许的最大时间间隙,
    超过则视为区间断开 (防跨数据空洞)。
    """
    segs = []
    start = None
    for i, m in enumerate(mask):
        if m and start is None:
            start = i
        elif not m and start is not None:
            segs.append((start, i - 1))
            start = None
    if start is not None:
        segs.append((start, len(mask) - 1))

    out = []
    for i0, i1 in segs:
        if i1 == i0:
            continue
        # 采样间隙检查
        if ts[i1] - ts[i0] < min_dur:
            continue
        gap_ok = True
        for k in range(i0, i1):
            if ts[k + 1] - ts[k] > max_gap:
                gap_ok = False
                break
        if not gap_ok:
            # 拆成子段
            sub_start = i0
            for k in range(i0, i1):
                if ts[k + 1] - ts[k] > max_gap:
                    if ts[k] - ts[sub_start] >= min_dur:
                        out.append((ts[sub_start], ts[k], sub_start, k))
                    sub_start = k + 1
            if ts[i1] - ts[sub_start] >= min_dur:
                out.append((ts[sub_start], ts[i1], sub_start, i1))
        else:
            out.append((ts[i0], ts[i1], i0, i1))
    return out


def overlap_segments(a_list, b_list):
    """两组区间 [(t0,t1,...)] 的时间交集列表 [(t0,t1)]。"""
    out = []
    i = j = 0
    a_list = sorted(a_list, key=lambda s: s[0])
    b_list = sorted(b_list, key=lambda s: s[0])
    while i < len(a_list) and j < len(b_list):
        a0, a1 = a_list[i][0], a_list[i][1]
        b0, b1 = b_list[j][0], b_list[j][1]
        lo, hi = max(a0, b0), min(a1, b1)
        if lo < hi:
            out.append((lo, hi))
        if a1 < b1:
            i += 1
        else:
            j += 1
    return out


def moving_average(x, k):
    """长度 k 的滑动平均 (k<=1 时原样返回)。"""
    x = np.asarray(x, dtype=float)
    if k <= 1 or len(x) < k:
        return x
    kernel = np.ones(k) / k
    pad = k // 2
    xp = np.pad(x, pad, mode="edge")
    return np.convolve(xp, kernel, mode="valid")[: len(x)]


def pose_delta_velocity(ts, xs, ys, yaws):
    """由位姿序列差分估计实测速度 (speed, wz)。

    重要: FastLIO 的 /Odometry 不填充 twist 字段 → /odom.twist 恒≈0,
    因此"实测速度"必须从 /odom 位姿差分获得 (2026-09-11 审查确认)。
    返回与输入等长的数组; 末点沿用前点值。
    """
    ts = np.asarray(ts, dtype=float)
    xs = np.asarray(xs, dtype=float)
    ys = np.asarray(ys, dtype=float)
    yaws = np.asarray(yaws, dtype=float)
    n = len(ts)
    v = np.zeros(n)
    w = np.zeros(n)
    if n < 2:
        return v, w
    for i in range(n - 1):
        dt = ts[i + 1] - ts[i]
        if dt <= 1e-6:
            continue
        v[i] = float(np.hypot(xs[i + 1] - xs[i], ys[i + 1] - ys[i]) / dt)
        dyaw = yaws[i + 1] - yaws[i]
        dyaw = (dyaw + np.pi) % (2.0 * np.pi) - np.pi
        w[i] = float(dyaw / dt)
    if n >= 3:
        v[-1] = v[-2]
        w[-1] = w[-2]
    return v, w


# ═══════════════════════════════════════════════════════════
#  bag 读取
# ═══════════════════════════════════════════════════════════


def resolve_bag_path(path):
    """支持: run 目录 / bag 目录 / 含 metadata.yaml 的目录。"""
    candidates = [
        path,
        os.path.join(path, "bag", "nav_debug"),
    ]
    for c in candidates:
        if os.path.isfile(os.path.join(c, "metadata.yaml")):
            return c
    # 最后一试: bag/ 下唯一子目录
    bag_parent = os.path.join(path, "bag")
    if os.path.isdir(bag_parent):
        subs = [os.path.join(bag_parent, d) for d in sorted(os.listdir(bag_parent))]
        subs = [s for s in subs if os.path.isfile(os.path.join(s, "metadata.yaml"))]
        if len(subs) == 1:
            return subs[0]
    return None


def stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def read_bag(bag_path):
    """读取 bag, 返回按类别整理的数据字典。"""
    try:
        from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
        from rclpy.serialization import deserialize_message
    except ImportError:
        print(
            "[ERROR] 无法导入 rosbag2_py / rclpy。请先 source ROS2 环境:\n"
            "  source scripts/ros2_source.sh",
            file=sys.stderr,
        )
        sys.exit(1)

    from nav_msgs.msg import Path, Odometry, OccupancyGrid
    from geometry_msgs.msg import Twist
    from visualization_msgs.msg import MarkerArray
    from tf2_msgs.msg import TFMessage
    from std_msgs.msg import Float32

    msg_classes = {
        "nav_msgs/msg/Path": Path,
        "geometry_msgs/msg/Twist": Twist,
        "nav_msgs/msg/Odometry": Odometry,
        "visualization_msgs/msg/MarkerArray": MarkerArray,
        "tf2_msgs/msg/TFMessage": TFMessage,
        "nav_msgs/msg/OccupancyGrid": OccupancyGrid,
        "std_msgs/msg/Float32": Float32,
    }

    # storage id 由目录内容推断
    has_db3 = any(f.endswith(".db3") for f in os.listdir(bag_path))
    storage_id = "sqlite3" if has_db3 else "mcap"

    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader.open(storage_options, converter_options)

    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}

    data = {
        "plans": [],        # (t, frame, pts Nx2)
        "trajs": [],        # (t, frame, pts_raw Nx2) 已合并 relay 输出与原始帧兜底
        "ref_plans": [],    # (t, frame, pts Nx2) MPPI 跟踪参考段 (transformed_global_plan)
        "cmd_raw": [],      # (t, vx, wz)
        "cmd_lim": [],      # (t, vx, wz)
        "odom": [],         # (t, x, y, yaw, vx_twist, wz_twist)
        "fl_odom": [],      # (t, x, y, yaw) FastLIO 原始 (camera_init 系, body 原点)
        "odom2map": [],     # (t, x, y, yaw) open3d ICP 矩阵
        "fitness": [],      # (t, value) ICP fitness
        "mode_events": [],  # (t, mode) 状态机切换事件
        "tf_map_odom": [],  # (t, x, y, yaw)
        "costmaps": [],     # (t, ox, oy, res, w, h, np.int8 data)
        "topic_counts": {},
        "warnings": [],
    }

    # 若已录到中继输出, 则忽略原始 /trajectories (避免同一帧重复计入)
    use_raw_traj = (
        "/nav_debug/mppi_optimal_traj" not in topics and "/trajectories" in topics
    )

    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        t_recv = t_ns * 1e-9
        if topic not in WANTED_TOPICS:
            continue
        exp_type = WANTED_TOPICS[topic]
        if topics.get(topic) != exp_type:
            warn = f"话题类型不符: {topic} 实际 {topics.get(topic)}, 期望 {exp_type}"
            if warn not in data["warnings"]:
                data["warnings"].append(warn)
            continue

        try:
            msg = deserialize_message(raw, msg_classes[exp_type])
        except Exception as e:  # noqa: BLE001
            data["warnings"].append(f"反序列化失败: {topic}: {e}")
            continue

        data["topic_counts"][topic] = data["topic_counts"].get(topic, 0) + 1

        if topic == "/plan":
            pts = np.array(
                [[p.pose.position.x, p.pose.position.y] for p in msg.poses], dtype=float
            )
            data["plans"].append((stamp_to_sec(msg.header.stamp), msg.header.frame_id, pts))
        elif topic == "/transformed_global_plan":
            pts = np.array(
                [[p.pose.position.x, p.pose.position.y] for p in msg.poses], dtype=float
            )
            data["ref_plans"].append(
                (stamp_to_sec(msg.header.stamp), msg.header.frame_id, pts)
            )
        elif topic == "/nav_debug/mppi_optimal_traj":
            pts = np.array(
                [[p.pose.position.x, p.pose.position.y] for p in msg.poses], dtype=float
            )
            data["trajs"].append((stamp_to_sec(msg.header.stamp), msg.header.frame_id, pts))
        elif topic == "/trajectories":
            # 兜底: 未录中继输出、直接录了原始 MarkerArray 时, 现场提取最优轨迹
            if not use_raw_traj:
                continue
            opt = [m for m in msg.markers if m.ns == OPTIMAL_NS]
            if not opt:
                continue
            opt.sort(key=lambda m: m.id)
            pts = np.array(
                [[m.pose.position.x, m.pose.position.y] for m in opt], dtype=float
            )
            data["trajs"].append((t_recv, opt[0].header.frame_id, pts))
        elif topic == "/cmd_vel":
            data["cmd_raw"].append((t_recv, msg.linear.x, msg.angular.z))
        elif topic == "/cmd_vel_limiter":
            data["cmd_lim"].append((t_recv, msg.linear.x, msg.angular.z))
        elif topic == "/odom":
            p = msg.pose.pose
            tw = msg.twist.twist
            data["odom"].append(
                (
                    stamp_to_sec(msg.header.stamp),
                    p.position.x,
                    p.position.y,
                    yaw_from_quaternion(
                        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
                    ),
                    float(np.hypot(tw.linear.x, tw.linear.y)),
                    tw.angular.z,
                )
            )
        elif topic == "/Odometry" or topic == "/odom2map":
            p = msg.pose.pose
            data["fl_odom" if topic == "/Odometry" else "odom2map"].append(
                (
                    stamp_to_sec(msg.header.stamp),
                    p.position.x,
                    p.position.y,
                    yaw_from_quaternion(
                        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
                    ),
                )
            )
        elif topic == "/localization_3d_confidence":
            data["fitness"].append((t_recv, float(msg.data)))
        elif topic in MODE_TOPICS:
            # 同一模式话题持续重发 (如 send_nav_goal -r 5 发 2s) 会产生多条,
            # 留原始事件, 去重在 compute_analysis 里做
            data["mode_events"].append((t_recv, MODE_TOPICS[topic]))
        elif topic == "/tf":
            for tr in msg.transforms:
                if tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                    q = tr.transform.rotation
                    data["tf_map_odom"].append(
                        (
                            stamp_to_sec(tr.header.stamp),
                            tr.transform.translation.x,
                            tr.transform.translation.y,
                            yaw_from_quaternion(q.x, q.y, q.z, q.w),
                        )
                    )
        elif topic == "/local_costmap/costmap":
            info = msg.info
            data["costmaps"].append(
                (
                    stamp_to_sec(msg.header.stamp),
                    info.origin.position.x,
                    info.origin.position.y,
                    info.resolution,
                    info.width,
                    info.height,
                    np.array(msg.data, dtype=np.int8),
                )
            )

    for key in (
        "plans",
        "trajs",
        "ref_plans",
        "cmd_raw",
        "cmd_lim",
        "odom",
        "fl_odom",
        "odom2map",
        "fitness",
        "mode_events",
        "tf_map_odom",
        "costmaps",
    ):
        data[key].sort(key=lambda row: row[0])

    return data


# ═══════════════════════════════════════════════════════════
#  分析
# ═══════════════════════════════════════════════════════════


def compute_analysis(
    data,
    stuck_eps,
    stuck_min_dur,
    zero_eps,
    zero_min_dur,
    tf_jump_xy=0.10,
    tf_jump_yaw_deg=2.0,
    fitness_th=0.9,
    shim_wz=0.15,
):
    """统一时间基准 + map 系变换 + 关键区间检测。"""
    res = {"warnings": list(data["warnings"])}

    # ---- 时间基准 ----
    all_t = []
    for key in (
        "plans",
        "trajs",
        "ref_plans",
        "cmd_raw",
        "cmd_lim",
        "odom",
        "fl_odom",
        "odom2map",
        "fitness",
        "mode_events",
        "tf_map_odom",
        "costmaps",
    ):
        if data[key]:
            all_t.append(data[key][0][0])
    t0 = min(all_t) if all_t else 0.0
    res["t0"] = t0

    tf_series = PoseSeries(
        [r[0] for r in data["tf_map_odom"]],
        [r[1] for r in data["tf_map_odom"]],
        [r[2] for r in data["tf_map_odom"]],
        [r[3] for r in data["tf_map_odom"]],
    )
    res["tf_series"] = tf_series
    res["has_tf"] = len(tf_series) > 0
    if not res["has_tf"]:
        res["warnings"].append(
            "bag 中无 map->odom TF — 无法统一到 map 系; 请确认 open3d_loc 在录包时运行"
        )

    def to_map(points, frame, t):
        """(odom 系点集, 时间) -> map 系点集; 失败返回 None。"""
        if points.size == 0:
            return None
        if frame == "map":
            return points
        if not res["has_tf"]:
            return None
        pose = tf_series.at(t)
        if pose is None:
            return None
        xo, yo, yaw, _ = pose
        return transform_points(points, xo, yo, yaw)

    # ---- 机器人轨迹 (map 系) ----
    odom_ts = np.array([r[0] for r in data["odom"]])
    robot = {"t": odom_ts, "t_rel": odom_ts - t0}
    if len(data["odom"]) > 0:
        xs = np.array([r[1] for r in data["odom"]])
        ys = np.array([r[2] for r in data["odom"]])
        yaws = np.array([r[3] for r in data["odom"]])
        robot["x_odom"], robot["y_odom"], robot["yaw_odom"] = xs, ys, yaws
        robot["vx_twist"] = np.array([r[4] for r in data["odom"]])
        robot["wz_twist"] = np.array([r[5] for r in data["odom"]])
        # 实测速度: 位姿差分。FastLIO 不填充 twist 字段 (/odom.twist 恒≈0),
        # 不能依赖 twist 做卡停检测 (2026-09-11 mount 审查确认)。
        robot["vx_meas"], robot["wz_meas"] = pose_delta_velocity(odom_ts, xs, ys, yaws)
        if (
            float(np.max(np.abs(robot["vx_twist"]))) < 0.01
            and float(np.max(robot["vx_meas"])) > 0.05
        ):
            res["warnings"].append(
                "/odom.twist 近似恒 0 (FastLIO 未填充 twist 字段), "
                "实测速度已改用 /odom 位姿差分"
            )

        if res["has_tf"]:
            xm = np.empty_like(xs)
            ym = np.empty_like(ys)
            yawm = np.empty_like(yaws)
            for i, t in enumerate(odom_ts):
                pose = tf_series.at(t)
                if pose is None:
                    continue
                xo, yo, yaw, _ = pose
                pt = transform_points(np.array([[xs[i], ys[i]]]), xo, yo, yaw)
                xm[i], ym[i] = pt[0]
                yawm[i] = yaw + yaws[i]
            robot["x_map"], robot["y_map"], robot["yaw_map"] = xm, ym, yawm

        # 卡停检测: 平滑后速度低于阈值
        dt = np.median(np.diff(odom_ts)) if len(odom_ts) > 1 else 0.1
        k = max(1, int(round(0.5 / max(dt, 1e-3))))  # ~0.5s 窗
        v_smooth = moving_average(robot["vx_meas"], k)
        robot["v_smooth"] = v_smooth
        stuck_mask = v_smooth < stuck_eps
        robot["stuck_segs"] = find_segments(stuck_mask, odom_ts, stuck_min_dur)
    else:
        robot["stuck_segs"] = []
        res["warnings"].append("未录到 /odom — 机器人轨迹与卡停检测不可用")

    res["robot"] = robot

    # ---- MPPI 局部轨迹 (同时保留 odom / map 两套坐标) ----
    traj_map = []
    n_frame_bad = 0
    for t, frame, pts in data["trajs"]:
        if pts.size == 0:
            continue
        pm = to_map(pts, frame, t)
        if pm is None:
            n_frame_bad += 1
            continue
        traj_map.append((t, pts, pm))
    res["traj_map"] = traj_map
    if n_frame_bad > 0:
        res["warnings"].append(f"{n_frame_bad} 帧 MPPI 轨迹无法变换到 map 系 (TF 缺失/越界)")
    if tf_series.n_clamped > 0:
        res["warnings"].append(
            f"map->odom TF 插值越界 {tf_series.n_clamped}/{tf_series.n_queries} 次 "
            "(用了端点值外推; 检查 TF 与数据时间覆盖是否一致)"
        )

    # ---- 全局路径 (确认 map 系) ----
    n_plan_bad = 0
    for _, frame, _ in data["plans"]:
        if frame and frame != "map":
            n_plan_bad += 1
    if n_plan_bad > 0:
        res["warnings"].append(f"{n_plan_bad} 条全局路径 frame_id 非 map, 注意对齐")

    # ---- 指令区间检测 ----
    lim_ts = np.array([r[0] for r in data["cmd_lim"]])
    if len(lim_ts) > 0:
        vx = np.array([r[1] for r in data["cmd_lim"]])
        wz = np.array([r[2] for r in data["cmd_lim"]])
        zero_mask = (np.abs(vx) < zero_eps) & (np.abs(wz) < zero_eps)
        res["zero_cmd_segs"] = find_segments(zero_mask, lim_ts, zero_min_dur)
        # 无平移指令: vx≈0 (wz 可非零, 如卡停后仍在原地转向)
        vx_mask = np.abs(vx) < zero_eps
        res["notrans_segs"] = find_segments(vx_mask, lim_ts, zero_min_dur)
        res["lim_ts"], res["lim_vx"], res["lim_wz"] = lim_ts, vx, wz
    else:
        res["zero_cmd_segs"] = []
        res["notrans_segs"] = []
        res["warnings"].append("未录到 /cmd_vel_limiter — 指令区间检测不可用")

    # ---- 与卡停求交 ----
    stuck_tt = [(s[0], s[1]) for s in robot["stuck_segs"]]
    try:
        res["zero_and_stuck"] = overlap_segments(
            [(s[0], s[1]) for s in res["zero_cmd_segs"]], stuck_tt
        )
        res["vx0_and_stuck"] = overlap_segments(
            [(s[0], s[1]) for s in res["notrans_segs"]], stuck_tt
        )
    except Exception:  # noqa: BLE001
        res["zero_and_stuck"] = []
        res["vx0_and_stuck"] = []

    # ---- MPPI 跟踪参考段 (取"最可疑时刻"附近一帧, 转 map 系供叠加图) ----
    res["ref_map"] = None
    if data["ref_plans"]:
        pick_t = data["ref_plans"][-1][0]
        seg0 = primary_seg(res)
        if seg0:
            pick_t = 0.5 * (seg0[0] + seg0[1])
        ref = min(data["ref_plans"], key=lambda c: abs(c[0] - pick_t))
        pm = to_map(ref[2], ref[1], ref[0])
        if pm is not None:
            res["ref_map"] = (ref[0], pm)

    # ---- map→odom TF 阶跃检测 (跳变瞬态是 MPPI 指令尖峰/路径瞬移的直接诱因) ----
    res["tf_jumps"] = []
    tf_rows = data["tf_map_odom"]
    for i in range(1, len(tf_rows)):
        dxy = float(np.hypot(tf_rows[i][1] - tf_rows[i - 1][1],
                             tf_rows[i][2] - tf_rows[i - 1][2]))
        dyaw = (tf_rows[i][3] - tf_rows[i - 1][3] + np.pi) % (2.0 * np.pi) - np.pi
        dyaw_deg = abs(float(np.degrees(dyaw)))
        # 两帧正常间隔 0.5s (ICP 2Hz); 间隔异常大时跳变可能只是长时间冻结后恢复,
        # 同样记录但在事件里带上间隔供人工判断
        dt = tf_rows[i][0] - tf_rows[i - 1][0]
        if dxy > tf_jump_xy or dyaw_deg > tf_jump_yaw_deg:
            res["tf_jumps"].append(
                (tf_rows[i][0], dxy, dyaw_deg, dt)
            )

    # ---- ICP fitness 统计与冻结区间 (fitness≤阈值 ⇒ mat_odom2map 冻结旧值) ----
    if data["fitness"]:
        ft = np.array([r[0] for r in data["fitness"]])
        fv = np.array([r[1] for r in data["fitness"]])
        res["fitness_stats"] = {
            "n": len(fv),
            "min": float(np.min(fv)),
            "mean": float(np.mean(fv)),
            "th": fitness_th,
            "low_ratio": float(np.mean(fv <= fitness_th)),
        }
        res["frozen_segs"] = find_segments(fv <= fitness_th, ft, 1.0)
    else:
        res["fitness_stats"] = None
        res["frozen_segs"] = []

    # ---- RotationShim 原地旋转推断 ----
    # 判据: MPPI 最优轨迹 (relay 输出, 正常 10Hz) 出现 >1.5s 的静默间隙,
    # 且该间隙内 /cmd_vel_limiter 的 wz 持续非零 (>shim_wz) —
    # Shim 接管时 MPPI 不更新轨迹, 表现为原地转向。
    res["shim_segs"] = []
    if data["trajs"] and len(res.get("lim_ts", [])) > 0:
        tt = [t for t, _, _ in data["trajs"]]
        for i in range(len(tt) - 1):
            t_a, t_b = tt[i], tt[i + 1]
            if t_b - t_a <= 1.5:
                continue
            idx = [j for j, t in enumerate(res["lim_ts"]) if t_a <= t <= t_b]
            if not idx:
                continue
            wz_seg = np.abs(res["lim_wz"][idx])
            if float(np.median(wz_seg)) > shim_wz:
                res["shim_segs"].append((t_a, t_b))

    # ---- 模式切换事件 (去重: 同模式 3s 内重复发布记一次) ----
    modes = []
    for t, m in data["mode_events"]:
        if modes and modes[-1][1] == m and t - modes[-1][0] < 3.0:
            continue
        modes.append((t, m))
    res["mode_events"] = modes

    return res


def primary_seg(res):
    """最可疑区间: 优先 vx≈0∩卡停, 退而求其次 指令全零∩卡停。"""
    if res.get("vx0_and_stuck"):
        return res["vx0_and_stuck"][0]
    if res.get("zero_and_stuck"):
        return res["zero_and_stuck"][0]
    return None


# ═══════════════════════════════════════════════════════════
#  输出: CSV
# ═══════════════════════════════════════════════════════════


def write_csv(path, header, rows):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)


def export_csvs(data, res, out_dir):
    t0 = res["t0"]
    paths = []

    robot = res["robot"]
    if len(robot.get("t", [])) > 0:
        has_map = "x_map" in robot
        header = ["t_rel_s", "t_unix_s"]
        if has_map:
            header += ["x_map", "y_map", "yaw_map"]
        header += ["x_odom", "y_odom", "vx_pose", "wz_pose", "vx_twist", "wz_twist"]
        rows = []
        for i, t in enumerate(robot["t"]):
            row = [f"{t - t0:.3f}", f"{t:.3f}"]
            if has_map:
                row += [
                    f"{robot['x_map'][i]:.4f}",
                    f"{robot['y_map'][i]:.4f}",
                    f"{robot['yaw_map'][i]:.4f}",
                ]
            row += [
                f"{robot['x_odom'][i]:.4f}",
                f"{robot['y_odom'][i]:.4f}",
                f"{robot['vx_meas'][i]:.4f}",
                f"{robot['wz_meas'][i]:.4f}",
                f"{robot['vx_twist'][i]:.4f}",
                f"{robot['wz_twist'][i]:.4f}",
            ]
            rows.append(row)
        p = os.path.join(out_dir, "robot_traj_map.csv")
        write_csv(p, header, rows)
        paths.append(p)

    for key, name in (("cmd_raw", "cmd_vel.csv"), ("cmd_lim", "cmd_vel_limiter.csv")):
        if not data[key]:
            continue
        rows = [[f"{t - t0:.3f}", f"{t:.3f}", f"{vx:.4f}", f"{wz:.4f}"] for t, vx, wz in data[key]]
        p = os.path.join(out_dir, name)
        write_csv(p, ["t_rel_s", "t_unix_s", "vx", "wz"], rows)
        paths.append(p)

    for key, name in (
        ("fl_odom", "fastlio_odom.csv"),
        ("odom2map", "odom2map.csv"),
    ):
        if not data[key]:
            continue
        rows = [
            [f"{t - t0:.3f}", f"{t:.3f}", f"{x:.4f}", f"{y:.4f}", f"{yaw:.4f}"]
            for t, x, y, yaw in data[key]
        ]
        p = os.path.join(out_dir, name)
        write_csv(p, ["t_rel_s", "t_unix_s", "x", "y", "yaw"], rows)
        paths.append(p)

    if data["fitness"]:
        rows = [[f"{t - t0:.3f}", f"{t:.3f}", f"{v:.4f}"] for t, v in data["fitness"]]
        p = os.path.join(out_dir, "icp_fitness.csv")
        write_csv(p, ["t_rel_s", "t_unix_s", "fitness"], rows)
        paths.append(p)

    if data["plans"]:
        rows = []
        for pid, (t, frame, pts) in enumerate(data["plans"]):
            for seq, (x, y) in enumerate(pts):
                rows.append([pid, f"{t - t0:.3f}", seq, f"{x:.4f}", f"{y:.4f}"])
        p = os.path.join(out_dir, "global_plan.csv")
        write_csv(p, ["plan_id", "t_rel_s", "seq", "x_map", "y_map"], rows)
        paths.append(p)

    if res["traj_map"]:
        rows = []
        for tid, (t, pts_raw, pts_map) in enumerate(res["traj_map"]):
            for seq in range(pts_map.shape[0]):
                xr = pts_raw[seq, 0] if seq < pts_raw.shape[0] else float("nan")
                yr = pts_raw[seq, 1] if seq < pts_raw.shape[0] else float("nan")
                rows.append(
                    [
                        tid,
                        f"{t - t0:.3f}",
                        seq,
                        f"{xr:.4f}",
                        f"{yr:.4f}",
                        f"{pts_map[seq, 0]:.4f}",
                        f"{pts_map[seq, 1]:.4f}",
                    ]
                )
        p = os.path.join(out_dir, "mppi_traj.csv")
        write_csv(p, ["traj_id", "t_rel_s", "seq", "x_odom", "y_odom", "x_map", "y_map"], rows)
        paths.append(p)

    return paths


# ═══════════════════════════════════════════════════════════
#  输出: 图 (matplotlib 可选)
# ═══════════════════════════════════════════════════════════


def make_plots(data, res, out_dir, max_traj_slices):
    try:
        os.environ.setdefault("MPLBACKEND", "Agg")
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.collections import LineCollection
        from matplotlib.lines import Line2D

        cmap = plt.get_cmap("viridis")

        # 中文字体回退 (缺失时仅影响图内文字渲染, 不影响数据)
        plt.rcParams["font.sans-serif"] = [
            "Noto Sans CJK SC",
            "WenQuanYi Zen Hei",
            "Source Han Sans SC",
            "PingFang SC",
            "Microsoft YaHei",
            "DejaVu Sans",
        ]
        plt.rcParams["axes.unicode_minus"] = False
    except ImportError:
        print(
            "[WARN] 未安装 matplotlib, 跳过 PNG 生成 (CSV/summary 不受影响)\n"
            "       安装: sudo apt install python3-matplotlib  或  pip3 install matplotlib"
        )
        return []

    t0 = res["t0"]
    robot = res["robot"]
    outputs = []

    # ══════ 图 1: map 系叠加 ══════
    fig, ax = plt.subplots(figsize=(11, 11))

    # -- 代价地图快照 (若有): 选"零指令∩卡停"第一段中点, 否则用最后一张。
    #    局部代价地图网格在 odom 系; 逐格取中心经 map->odom TF 转 map 系后散点绘制
    #    (不用 imshow: 网格轴对齐于 odom, 若 map->odom 含旋转会错位)。
    cm_used_t = None
    if data["costmaps"] and res.get("has_tf"):
        pick_t = data["costmaps"][-1][0]
        seg0 = primary_seg(res)
        if seg0:
            pick_t = 0.5 * (seg0[0] + seg0[1])
        cm = min(data["costmaps"], key=lambda c: abs(c[0] - pick_t))
        t_cm, ox, oy, res_m, w, h, occ = cm
        pose_cm = res["tf_series"].at(t_cm)
        if pose_cm is not None:
            xo, yo, yaw_tf, _ = pose_cm
            cm_used_t = t_cm - t0
            arr = occ.reshape(h, w)
            for mask_cond, mk_color, mk_alpha, mk_size in (
                (arr >= 65, "red", 0.85, 16),                      # 障碍
                ((arr >= 1) & (arr < 65), "orange", 0.30, 8),      # 膨胀/低代价
            ):
                ii, jj = np.where(mask_cond)
                if len(ii) == 0:
                    continue
                cx = ox + (jj + 0.5) * res_m
                cy = oy + (ii + 0.5) * res_m
                pts_cm = transform_points(np.column_stack([cx, cy]), xo, yo, yaw_tf)
                ax.scatter(
                    pts_cm[:, 0],
                    pts_cm[:, 1],
                    s=mk_size,
                    c=mk_color,
                    alpha=mk_alpha,
                    marker="s",
                    linewidths=0,
                    zorder=0,
                )

    # -- 全局路径 (橙色; 最新一条最深) --
    for i, (t, frame, pts) in enumerate(data["plans"]):
        if pts.size == 0:
            continue
        last = i == len(data["plans"]) - 1
        ax.plot(
            pts[:, 0],
            pts[:, 1],
            color="tab:orange",
            lw=2.2 if last else 1.0,
            alpha=0.95 if last else 0.35,
            zorder=3,
        )

    # -- 机器人轨迹 (按时间着色) --
    if "x_map" in robot and len(robot["x_map"]) > 1:
        pts = np.column_stack([robot["x_map"], robot["y_map"], robot["t_rel"]])
        segs = np.stack([pts[:-1, :2], pts[1:, :2]], axis=1)
        lc = LineCollection(segs, cmap=cmap, zorder=5)
        lc.set_array(pts[:-1, 2])
        lc.set_linewidth(2.5)
        ax.add_collection(lc)
        cb = fig.colorbar(lc, ax=ax, fraction=0.035, pad=0.02)
        cb.set_label("时间 t (s, 相对开录)")

        # 起始/结束标记
        ax.plot(pts[0, 0], pts[0, 1], marker=">", color="limegreen", ms=14,
                mec="k", zorder=8, label=None)
        ax.plot(pts[-1, 0], pts[-1, 1], marker="s", color="k", ms=9, zorder=8)

    # -- MPPI 最优局部轨迹 (均匀抽帧, 与机器人轨迹同色标) --
    trajs = res["traj_map"]
    if trajs:
        n = len(trajs)
        step = max(1, n // max_traj_slices)
        t_end = max([t for t, _, _ in trajs]) - t0
        t_end = max(t_end, 1e-6)
        for t, _pts_raw, pts in trajs[::step]:
            if pts.shape[0] < 2:
                continue
            ax.plot(
                pts[:, 0],
                pts[:, 1],
                color=cmap(min((t - t0) / t_end, 1.0)),
                lw=1.1,
                alpha=0.65,
                zorder=4,
            )

    # -- MPPI 跟踪参考段 (transformed_global_plan, "最可疑时刻"一帧, 虚线) --
    if res.get("ref_map") is not None:
        _, pts_ref = res["ref_map"]
        if pts_ref.shape[0] >= 2:
            ax.plot(
                pts_ref[:, 0],
                pts_ref[:, 1],
                color="deepskyblue",
                ls="--",
                lw=1.6,
                alpha=0.9,
                zorder=4,
            )

    # -- 限速后指令箭头 (每秒抽样: 方向=车头, 长度 ∝ vx; vx≈0 画橙点) --
    if len(data["cmd_lim"]) > 0 and "x_map" in robot:
        odom_ts = robot["t"]
        last_t = -1e9
        for t, vx, _wz in data["cmd_lim"]:
            if t - last_t < 1.0:
                continue
            last_t = t
            i = int(np.argmin(np.abs(odom_ts - t)))
            x, y, yaw = robot["x_map"][i], robot["y_map"][i], robot["yaw_map"][i]
            if abs(vx) < 0.05:
                ax.plot(x, y, marker="o", color="orange", ms=4.5, zorder=6)
                continue
            L = float(np.clip(vx * 2.0, -0.8, 0.8))
            dx, dy = L * np.cos(yaw), L * np.sin(yaw)
            color = "royalblue" if vx > 0 else "red"
            ax.annotate(
                "",
                xy=(x + dx, y + dy),
                xytext=(x, y),
                arrowprops=dict(arrowstyle="-|>", color=color, lw=1.4, alpha=0.85),
                zorder=6,
            )

    # -- 卡停 / 无平移指令标记 --
    if res["vx0_and_stuck"] and "x_map" in robot:
        for seg in res["vx0_and_stuck"]:
            tm = 0.5 * (seg[0] + seg[1])
            i = int(np.argmin(np.abs(robot["t"] - tm)))
            ax.plot(
                robot["x_map"][i],
                robot["y_map"][i],
                marker="x",
                color="red",
                ms=12,
                mew=3,
                zorder=9,
            )
    if robot["stuck_segs"] and "x_map" in robot:
        for seg in robot["stuck_segs"]:
            tm = 0.5 * (seg[0] + seg[1])
            i = int(np.argmin(np.abs(robot["t"] - tm)))
            ax.plot(
                robot["x_map"][i],
                robot["y_map"][i],
                marker="o",
                color="tab:blue",
                ms=8,
                mfc="none",
                mew=2,
                zorder=9,
            )

    # -- 图例 / 标注 --
    handles = [
        Line2D([], [], color="tab:orange", lw=2.2, label="全局路径 /plan (map)"),
        Line2D([], [], color="deepskyblue", ls="--", lw=1.6, label="MPPI 跟踪参考段"),
        Line2D([], [], color="gray", lw=2.5, label="机器人轨迹 (按时间着色)"),
        Line2D([], [], color="gray", lw=1.1, alpha=0.65, label="MPPI 局部轨迹"),
        Line2D([], [], marker=">", color="limegreen", ls="", ms=12, mec="k", label="起点"),
        Line2D([], [], marker="s", color="k", ls="", ms=8, label="终点"),
        Line2D([], [], marker="x", color="red", ls="", ms=10, mew=3, label="vx≈0 ∩ 机器人卡停"),
        Line2D([], [], marker="o", color="tab:blue", ls="", ms=8, mfc="none", mew=2, label="机器人卡停"),
        Line2D([], [], marker="o", color="orange", ls="", ms=4.5, label="vx≈0 采样点"),
    ]
    if cm_used_t is not None:
        handles += [
            Line2D([], [], marker="s", color="red", ls="", ms=6, alpha=0.85,
                   label="障碍单元 (代价图快照)"),
            Line2D([], [], marker="s", color="orange", ls="", ms=5, alpha=0.5,
                   label="膨胀/低代价"),
        ]
    ax.legend(handles=handles, loc="best", fontsize=9)
    title = "导航调试回放 (map 系)"
    if cm_used_t is not None:
        title += f" — 含 {cm_used_t:.1f}s 时刻局部代价地图快照"
    ax.set_title(title)
    ax.text(
        0.01,
        0.01,
        "箭头 = 限速后指令 vx (蓝: 前进 / 红: 后退, 长度∝|vx|); 橙点 = 指令≈0; wz 见 cmd_vel.png",
        transform=ax.transAxes,
        fontsize=8,
        color="dimgray",
    )
    ax.set_xlabel("x (map, m)")
    ax.set_ylabel("y (map, m)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    p = os.path.join(out_dir, "map_overlay.png")
    fig.savefig(p, dpi=150)
    plt.close(fig)
    outputs.append(p)

    # ══════ 图 2: 指令时序 (+ fitness/TF跳变/模式标记) ══════
    n_rows = 3 if data["fitness"] else 2
    fig, axes = plt.subplots(n_rows, 1, figsize=(12, 3.2 * n_rows + 1), sharex=True)
    if n_rows == 2:
        ax1, ax2 = axes
        axf = None
    else:
        ax1, ax2, axf = axes
    all_axes = [a for a in (ax1, ax2, axf) if a is not None]

    # 零指令区间底色
    for seg in res["zero_cmd_segs"]:
        for axx in all_axes:
            axx.axvspan(seg[0] - t0, seg[1] - t0, color="red", alpha=0.10, lw=0)

    if data["cmd_raw"]:
        tr = np.array([r[0] for r in data["cmd_raw"]]) - t0
        ax1.plot(tr, [r[1] for r in data["cmd_raw"]], color="gray", ls="--",
                 lw=1.0, label="/cmd_vel (MPPI 原始)")
        ax2.plot(tr, [r[2] for r in data["cmd_raw"]], color="gray", ls="--", lw=1.0)
    if data["cmd_lim"]:
        tl = np.array([r[0] for r in data["cmd_lim"]]) - t0
        ax1.plot(tl, [r[1] for r in data["cmd_lim"]], color="red", lw=1.6,
                 label="/cmd_vel_limiter (下发 RL)")
        ax2.plot(tl, [r[2] for r in data["cmd_lim"]], color="red", lw=1.6)
    if len(robot.get("t", [])) > 0:
        tro = robot["t_rel"]
        ax1.plot(tro, robot["vx_meas"], color="tab:blue", lw=1.2, alpha=0.85,
                 label="/odom 实测速度 (位姿差分)")
        ax2.plot(tro, robot["wz_meas"], color="tab:blue", lw=1.2, alpha=0.85)

    # map→odom TF 跳变竖线 (紫色点划线) — 与 vx 骤降/wz 尖峰时刻对齐即 H2 证据
    for tj, dxy, dyaw_deg, _dt in res["tf_jumps"]:
        for axx in all_axes:
            axx.axvline(tj - t0, color="purple", ls="-.", lw=1.0, alpha=0.75)

    # 模式切换标记 (顶部三角)
    for tm, mode in res["mode_events"]:
        ax1.plot(tm - t0, ax1.get_ylim()[1], marker="^", color="black", ms=8,
                 clip_on=False)

    # ICP fitness 行
    if axf is not None:
        tf_arr = np.array([r[0] for r in data["fitness"]]) - t0
        fv_arr = np.array([r[1] for r in data["fitness"]])
        axf.plot(tf_arr, fv_arr, color="seagreen", lw=1.2, label="ICP fitness")
        if res.get("fitness_stats"):
            axf.axhline(res["fitness_stats"]["th"], color="seagreen", ls=":",
                        lw=1.0, alpha=0.8)
            axf.text(
                tf_arr[0] if len(tf_arr) else 0,
                res["fitness_stats"]["th"] + 0.01,
                f" th={res['fitness_stats']['th']} (低于则冻结 map→odom)",
                fontsize=7, color="seagreen",
            )
        for seg in res.get("frozen_segs", []):
            axf.axvspan(seg[0] - t0, seg[1] - t0, color="seagreen", alpha=0.12, lw=0)
        axf.set_ylabel("fitness")
        axf.set_ylim(0.0, 1.05)
        axf.grid(True, alpha=0.3)

    ax1.set_ylabel("vx (m/s)")
    ax2.set_ylabel("wz (rad/s)")
    axes[-1].set_xlabel("时间 t (s, 相对开录)")
    ax1.legend(loc="upper right", fontsize=9)
    ax1.set_title(
        "速度指令链: MPPI 原始 -> 限速器 -> RL / 实测\n"
        "红底=指令≈0 | 紫点划线=map→odom 跳变 | 黑三角=模式切换"
        + (" | 绿行=ICP fitness(绿底=冻结)" if axf is not None else "")
    )
    for axx in all_axes:
        axx.grid(True, alpha=0.3)
    fig.tight_layout()
    p = os.path.join(out_dir, "cmd_vel.png")
    fig.savefig(p, dpi=150)
    plt.close(fig)
    outputs.append(p)

    return outputs


# ═══════════════════════════════════════════════════════════
#  输出: 摘要
# ═══════════════════════════════════════════════════════════


def _fmt_t(t, t0):
    return f"{t - t0:7.1f}s"


def _robot_map_at(robot, t):
    """机器人 t 时刻的 map 坐标字符串 (无则 '?')。"""
    if "x_map" not in robot or len(robot["t"]) == 0:
        return "?"
    i = int(np.argmin(np.abs(robot["t"] - t)))
    return f"({robot['x_map'][i]:.2f}, {robot['y_map'][i]:.2f})"


def write_summary(data, res, run_name, out_path):
    t0 = res["t0"]
    robot = res["robot"]
    lines = []
    lines.append(f"# 导航调试日志自动摘要 — {run_name}")
    lines.append("")
    lines.append(f"- 生成时间: {datetime.now().isoformat(timespec='seconds')}")
    lines.append(f"- 时间基准 t0 (unix): {t0:.3f}")

    # 数据计数
    lines.append("")
    lines.append("## 数据计数")
    lines.append("")
    lines.append("| 话题 | 帧数 |")
    lines.append("|---|---|")
    for topic in WANTED_TOPICS:
        lines.append(f"| `{topic}` | {data['topic_counts'].get(topic, 0)} |")

    # 核心结论
    lines.append("")
    lines.append("## 关键区间 (相对开录时间)")
    lines.append("")
    zs = res["zero_cmd_segs"]
    ss = robot["stuck_segs"]
    vxs = res["vx0_and_stuck"]
    zss = res["zero_and_stuck"]

    def _cmd_stats_in(seg):
        """卡停区间内 /cmd_vel_limiter 统计: (|vx|max, wz_min, wz_max)。"""
        ws = [(t, v, w) for t, v, w in data["cmd_lim"] if seg[0] <= t <= seg[1]]
        if not ws:
            return None
        return (
            max(abs(v) for _, v, _ in ws),
            min(w for _, _, w in ws),
            max(w for _, _, w in ws),
        )

    if ss:
        lines.append(f"**机器人卡停区间 ({len(ss)} 段)** — `/odom` 平移速度低于阈值:")
        lines.append("")
        for seg in ss:
            info = _cmd_stats_in(seg)
            extra = ""
            if info:
                vx_max, wz_min, wz_max = info
                extra = (
                    f", 期间指令 |vx|max={vx_max:.2f} m/s"
                    f", wz∈[{wz_min:+.2f}, {wz_max:+.2f}] rad/s"
                )
            lines.append(
                f"- {_fmt_t(seg[0], t0)} ~ {_fmt_t(seg[1], t0)} "
                f"(持续 {seg[1] - seg[0]:.1f}s), "
                f"位置 {_robot_map_at(robot, 0.5 * (seg[0] + seg[1]))}{extra}"
            )
        lines.append("")
    else:
        lines.append("- 未检测到机器人卡停 (`/odom` 平移速度未低于阈值)。")
        lines.append("")

    if vxs:
        lines.append("**★★ 卡停 ∩ vx≈0 (无平移指令, 最可疑):**")
        lines.append("")
        for seg in vxs:
            lines.append(
                f"- {_fmt_t(seg[0], t0)} ~ {_fmt_t(seg[1], t0)} "
                f"(持续 {seg[1] - seg[0]:.1f}s), "
                f"位置 {_robot_map_at(robot, 0.5 * (seg[0] + seg[1]))}"
            )
        lines.append("")
        lines.append(
            "> 在上述区间: 下发给 RL 的平移指令 (vx) 已归零、机器人未移动 — "
            "对应\"原地踏步\"现象 (是否同时在原地转向, 见上表 wz 统计)。"
            "打开 map_overlay.png 查看该点周围: 代价地图是否被障碍/膨胀堵住、"
            "MPPI 局部轨迹是否仍在尝试绕行/转向。"
        )
        lines.append("")
        if zss and len(zss) != len(vxs):
            lines.append(f"- 其中 wz 也≈0 (指令完全静止) 的区间: {len(zss)} 段")
            lines.append("")
    else:
        lines.append("**未检测到\"卡停 ∩ vx≈0\"的重叠区间。**")
        lines.append("")
        if zs:
            lines.append(f"- 供参考: 指令≈0 (vx与wz均≈0) 区间 {len(zs)} 段:")
            for seg in zs:
                lines.append(
                    f"  - {_fmt_t(seg[0], t0)} ~ {_fmt_t(seg[1], t0)}"
                    f" (持续 {seg[1] - seg[0]:.1f}s)"
                )
            lines.append("")

    # 轨迹统计
    if "x_map" in robot and len(robot["x_map"]) > 1:
        d = np.hypot(np.diff(robot["x_map"]), np.diff(robot["y_map"]))
        lines.append("## 轨迹统计")
        lines.append("")
        lines.append(f"- 机器人行驶距离 (map 系): {d.sum():.2f} m")
        lines.append(
            f"- 起点 ({robot['x_map'][0]:.2f}, {robot['y_map'][0]:.2f}) "
            f"-> 终点 ({robot['x_map'][-1]:.2f}, {robot['y_map'][-1]:.2f})"
        )
    if data["plans"]:
        last_len = (
            np.sum(np.hypot(*np.diff(data["plans"][-1][2], axis=0).T))
            if data["plans"][-1][2].shape[0] > 1
            else 0.0
        )
        lines.append(f"- 全局路径: {len(data['plans'])} 条 (重规划 {len(data['plans']) - 1} 次), 末条长度 {last_len:.2f} m")
    if res["traj_map"]:
        t_first = res["traj_map"][0][0]
        t_last = res["traj_map"][-1][0]
        span = t_last - t_first
        hz = len(res["traj_map"]) / span if span > 1e-6 else 0.0
        lines.append(f"- MPPI 局部轨迹: {len(res['traj_map'])} 帧 (~{hz:.1f} Hz)")
    lines.append("")

    # 定位质量 (map→odom / ICP)
    lines.append("## 定位质量 (map→odom / ICP)")
    lines.append("")
    if res["fitness_stats"]:
        fs = res["fitness_stats"]
        lines.append(
            f"- ICP fitness: n={fs['n']}, min={fs['min']:.3f}, mean={fs['mean']:.3f}; "
            f"低于 {fs['th']} 的占比 {fs['low_ratio'] * 100:.0f}%"
        )
        if res["frozen_segs"]:
            lines.append(
                f"- **冻结区间 (fitness≤阈值, map→odom 保持旧值): {len(res['frozen_segs'])} 段**:"
            )
            for seg in res["frozen_segs"]:
                lines.append(f"  - {_fmt_t(seg[0], t0)} ~ {_fmt_t(seg[1], t0)} (持续 {seg[1] - seg[0]:.1f}s)")
        else:
            lines.append("- 无 fitness 冻结区间")
    else:
        lines.append("- 未录到 /localization_3d_confidence (旧录包或未加此话题)")
    if res["tf_jumps"]:
        lines.append(
            f"- **map→odom 跳变 {len(res['tf_jumps'])} 次** (平移>阈值或旋转>阈值):"
        )
        for tj, dxy, dyaw_deg, dt in res["tf_jumps"][:20]:
            lines.append(
                f"  - {_fmt_t(tj, t0)}: Δxy={dxy * 100:.0f}cm Δyaw={dyaw_deg:.1f}° "
                f"(帧间隔 {dt:.2f}s)"
            )
        if len(res["tf_jumps"]) > 20:
            lines.append(f"  - ... 共 {len(res['tf_jumps'])} 次, 详见 odom2map.csv")
        lines.append(
            "> 跳变时刻若与 vx 骤降/wz 尖峰对齐 (cmd_vel.png 紫色点划线), "
            "则路径瞬移是卡停/原地转的直接诱因 (H2)。"
        )
    else:
        lines.append("- map→odom 无显著跳变")
    lines.append("")

    # RotationShim 原地旋转
    lines.append("## RotationShim 原地旋转区间")
    lines.append("")
    if res["shim_segs"]:
        lines.append(
            f"**检测到 {len(res['shim_segs'])} 段\"wz 持续非零 且 MPPI 轨迹静默\"** "
            "(Shim 接管, MPPI 不更新; 表现为原地转向):"
        )
        for seg in res["shim_segs"]:
            lines.append(f"- {_fmt_t(seg[0], t0)} ~ {_fmt_t(seg[1], t0)} (持续 {seg[1] - seg[0]:.1f}s)")
        lines.append("")
    else:
        lines.append("- 未检测到 (wz 非零期间 MPPI 轨迹仍在更新, 或无该数据)")
        lines.append("")

    # 模式切换事件
    lines.append("## 模式切换事件")
    lines.append("")
    if res["mode_events"]:
        for tm, m in res["mode_events"]:
            lines.append(f"- {_fmt_t(tm, t0)}: → **{m}**")
        lines.append("")
        lines.append(
            "> 卡停区间内若出现 → stand/keep, 指令归零是状态机行为而非 MPPI 问题。"
        )
    else:
        lines.append("- 无模式切换 (全程 walk 或未录模式话题)")
    lines.append("")

    # 数据质量
    lines.append("## 数据质量 / 备注")
    lines.append("")
    if res["warnings"]:
        for w in res["warnings"]:
            lines.append(f"- ⚠ {w}")
    else:
        lines.append("- 无告警")
    tf_ser = res.get("tf_series")
    if tf_ser is not None and len(tf_ser) > 0:
        lines.append(f"- map->odom TF 覆盖: {len(tf_ser)} 帧")
    lines.append("")

    # 排查提示
    lines.append("## 排查提示 (按链路分段)")
    lines.append("")
    lines.append("1. `/cmd_vel` 已≈0, `/cmd_vel_limiter`≈0 -> MPPI 输出的就是零速: 检查 map_overlay.png 中该点的代价地图与局部轨迹"
                 "（堵死/局部极小/目标方向被挡；RotationShim 原地旋转阶段 MPPI 不更新）。")
    lines.append("2. `/cmd_vel` 非零而 `/cmd_vel_limiter`≈0 -> 问题在 odom_bridge 限速器 (max_ax/max_az 参数)。")
    lines.append("3. `/cmd_vel_limiter` 非零而 `/odom` 实测速度≈0 -> 问题在下位机/RL 步态 (被障碍挡住、打滑、步态未启动; 注意 vx<0.05 死区只踏步)。")
    lines.append("4. map 系下轨迹与全局路径突然错位/跳变 -> 对照上方\"map→odom 跳变\"时刻表 (H2)。")
    lines.append("5. 原地转向 (wz 恒值) 且轨迹静默 -> RotationShim 接管 (H4), 查路径方向为何相对车头反转。")
    lines.append("6. /odom 与 /Odometry (fastlio_odom.csv) 趋势不一致 -> 问题在 odom_bridge (mount_rpy/高度偏移)。")
    lines.append("")

    with open(out_path, "w") as f:
        f.write("\n".join(lines))


# ═══════════════════════════════════════════════════════════
#  main
# ═══════════════════════════════════════════════════════════


def main():
    parser = argparse.ArgumentParser(
        description="导航调试日志后处理: 统一 map 系可视化 + CSV + 摘要",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("run", help="run 目录 (reports/nav_debug/...) 或 bag 目录")
    parser.add_argument("--analysis-dir", default=None, help="输出目录 (默认 <run>/analysis)")
    parser.add_argument("--no-plots", action="store_true", help="跳过 PNG (无 matplotlib 时)")
    parser.add_argument("--stuck-eps", type=float, default=0.05, help="卡停速度阈值 (m/s)")
    parser.add_argument("--stuck-min-dur", type=float, default=1.0, help="卡停最小时长 (s)")
    parser.add_argument("--zero-eps", type=float, default=0.05, help="指令≈0 阈值 (m/s, rad/s)")
    parser.add_argument("--zero-min-dur", type=float, default=1.0, help="指令≈0 最小时长 (s)")
    parser.add_argument("--tf-jump-xy", type=float, default=0.10, help="map→odom 跳变判定: 平移阈值 (m)")
    parser.add_argument("--tf-jump-yaw-deg", type=float, default=2.0, help="map→odom 跳变判定: 旋转阈值 (度)")
    parser.add_argument("--fitness-th", type=float, default=0.9, help="ICP fitness 冻结阈值")
    parser.add_argument("--shim-wz", type=float, default=0.15, help="RotationShim 推断的 wz 阈值 (rad/s)")
    parser.add_argument("--max-traj-slices", type=int, default=25, help="叠加图中 MPPI 轨迹抽帧数")
    args = parser.parse_args()

    bag_path = resolve_bag_path(args.run)
    if bag_path is None:
        print(f"[ERROR] 在 {args.run} 下找不到 bag (需要 metadata.yaml)", file=sys.stderr)
        sys.exit(1)

    run_name = os.path.basename(os.path.abspath(args.run))
    out_dir = args.analysis_dir or (
        os.path.join(args.run, "analysis") if os.path.isdir(args.run) else "analysis"
    )
    os.makedirs(out_dir, exist_ok=True)

    print(f"[nav_debug_plot] bag: {bag_path}")
    data = read_bag(bag_path)
    res = compute_analysis(
        data,
        args.stuck_eps,
        args.stuck_min_dur,
        args.zero_eps,
        args.zero_min_dur,
        tf_jump_xy=args.tf_jump_xy,
        tf_jump_yaw_deg=args.tf_jump_yaw_deg,
        fitness_th=args.fitness_th,
        shim_wz=args.shim_wz,
    )

    # CSV
    data_dir = os.path.join(out_dir, "data")
    os.makedirs(data_dir, exist_ok=True)
    csvs = export_csvs(data, res, data_dir)
    for p in csvs:
        print(f"  [csv] {p}")

    # 图
    if not args.no_plots:
        for p in make_plots(data, res, out_dir, args.max_traj_slices):
            print(f"  [png] {p}")

    # 摘要
    summary_path = os.path.join(out_dir, "summary.md")
    write_summary(data, res, run_name, summary_path)
    print(f"  [md ] {summary_path}")
    print("[nav_debug_plot] 完成")
    print(f"  建议先看: {summary_path} → {out_dir}/map_overlay.png / cmd_vel.png")


if __name__ == "__main__":
    main()
