#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
loco_perf_test.py — 运动控制 (MuJoCo 仿真) 基础性能测试执行器

对 /cmd_vel_limiter 发送 vx/vy/wz 指令, 从 /mujoco/ground_truth 采集真值,
测量并输出以下四类基础性能 (与导航需求对齐):

  1. straight  直线行走: 最大速度(不摔倒), 速度达成率, 是否走直线, 偏移率
  2. turn      原地转弯: 最大角速度(不摔倒), 转速达成率, 是否偏移原地, 偏移率
  3. arc       弧线行走: 最大执行速度(不摔倒), 速度/角速度达成率, 半径误差/轨迹残差
  4. stop      停止: 从 0.4 m/s 到静止的最大减速度与停止时间 (多次重复取统计)

用法 (仿真已启动且已进入 walk 模式, 或由 run_loco_perf.sh 一键编排):
  python3 scripts/loco_perf_test.py --test straight --report-dir reports
  python3 scripts/loco_perf_test.py --test stop --stop-speeds 0.4 --stop-reps 5
  python3 scripts/loco_perf_test.py --selftest        # 无 ROS 环境自检(合成数据)

指标口径 (详见 doc/motion_control_perf_test.md):
  - 达成率 = 稳态窗口实测均值 / 指令值
  - 直线偏移率 = 测量段最大横向偏差 / 行驶距离; 航向漂移率 = 航向角变化/距离 (°/m)
  - 原地偏移率 = 距起点最大位移 / 转过角度 (m/rad); 是否偏移原地 = 最大位移阈值
  - 弧线: Kasa 圆拟合半径误差 + 轨迹到拟合圆残差 RMS
  - 停止: 零指令时刻→亚阈值(0.05m/s持续0.1s)时刻为停止时间; 减速度取 v(t) 微分的峰值/均值
  - 摔倒判据 (与 nav_test_runner 一致): z<0.35m 或 |roll|>45° 或 |pitch|>45°

退出码: 0=完成; 2=完成但测试中出现摔倒; 3=中断/前置条件不满足
"""

import argparse
import csv
import json
import math
import os
import sys
import time
from datetime import datetime

import numpy as np

# ═══════════════════════════════════════════════════════════
#  常量
# ═══════════════════════════════════════════════════════════

FALL_Z_M = 0.35                 # 摔倒判据: 基座高度阈值 (m)
FALL_ANGLE_DEG = 45.0           # 摔倒判据: |roll|/|pitch| 阈值 (°)
CMD_TOPIC = "/cmd_vel_limiter"  # 外部速度指令入口 (ControlModule 订阅)
GT_TOPIC = "/mujoco/ground_truth"
GT_STALE_S = 3.0                # GT 断流判定 (wall 秒)
STOP_V_THRESH = 0.05            # 停止判定速度阈值 (m/s)
STOP_SUSTAIN_S = 0.10           # 停止判定需持续时间 (s)
SMOOTH_W_POS = 51               # 位置平滑窗口 (采样点, 1kHz 下=51ms)
SMOOTH_W_YAW = 31               # 姿态平滑窗口

# 判定阈值 (初值 — 跑完一轮基线后按实测校准, 见协议文档)
V_STRAIGHT_HEADING_RATE_MAX = 3.0    # °/m
V_STRAIGHT_LATERAL_RATIO_MAX = 0.05  # 5%
V_TURN_INPLACE_DMAX_MAX = 0.15       # m
V_TURN_DRIFT_PER_RAD_MAX = 0.05      # m/rad
V_ARC_RADIUS_ERR_MAX = 0.25          # 25%
V_ARC_RESID_RMS_MAX = 0.10           # m

DEFAULT_LEVELS = {
    "straight": "0.2,0.4,0.6,0.8,1.0,1.2,1.4",
    "turn": "0.2,0.4,0.6,0.8,1.0,1.2,1.4",
    # 弧线: vx:wz 成对, 默认按 R=1m 恒定半径同步递增
    "arc": "0.2:0.2,0.4:0.4,0.6:0.6,0.8:0.8,1.0:1.0",
    "stop": "0.4",
}

# ═══════════════════════════════════════════════════════════
#  指标计算核心 (纯函数, 可脱离 ROS 自检)
# ═══════════════════════════════════════════════════════════

def smooth_series(x, w):
    """中心滑动平均; 边缘复制填充 (w 为窗长, 采样点数)"""
    x = np.asarray(x, dtype=float)
    if w <= 1 or len(x) < 3:
        return x
    pad = w // 2
    xp = np.pad(x, (pad, pad), mode="edge")
    k = np.ones(w, dtype=float) / w
    return np.convolve(xp, k, mode="valid")


def speed_profile(t, x, y, w=SMOOTH_W_POS):
    """由 GT 位置差分求基座速度 (平滑后中心差分)"""
    xs, ys = smooth_series(x, w), smooth_series(y, w)
    vx = np.gradient(xs, t)
    vy = np.gradient(ys, t)
    return vx, vy, np.hypot(vx, vy)


def yaw_rate_profile(t, yaw, w=SMOOTH_W_YAW):
    """由 GT 偏航角求偏航角速度 (先解缠再平滑)"""
    yu = np.unwrap(np.asarray(yaw, dtype=float))
    yus = smooth_series(yu, w)
    wz = np.gradient(yus, t)
    return yu, wz


def path_length(x, y):
    x, y = np.asarray(x, float), np.asarray(y, float)
    if len(x) < 2:
        return 0.0
    return float(np.sum(np.hypot(np.diff(x), np.diff(y))))


def ref_pose(t, x, y, yaw, t_ref_span=0.25):
    """取测量段开头 t_ref_span 秒的中位位姿作为参考"""
    m = t <= (t[0] + t_ref_span)
    if not np.any(m):
        m = np.ones_like(t, dtype=bool)
    return float(np.median(x[m])), float(np.median(y[m])), float(np.median(yaw[m]))


def kasa_circle_fit(x, y):
    """Kasa 代数圆拟合; 返回 (cx, cy, r, 残差RMS, 残差数组) 或 None"""
    x, y = np.asarray(x, float), np.asarray(y, float)
    if len(x) < 8:
        return None
    A = np.column_stack([2 * x, 2 * y, np.ones(len(x))])
    b = x * x + y * y
    try:
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    except np.linalg.LinAlgError:
        return None
    cx, cy, c = sol
    r2 = c + cx * cx + cy * cy
    if r2 <= 0:
        return None
    r = math.sqrt(r2)
    d = np.hypot(x - cx, y - cy)
    res = d - r
    return float(cx), float(cy), float(r), float(np.sqrt(np.mean(res ** 2))), res


def _guard_short(t):
    return len(t) < 8


def metrics_straight(t, x, y, yaw, t_win0, cmd_vx):
    """直线段指标。t/x/y/yaw: 测量段数组(自 level 稳定后起); t_win0: 稳态窗口起点(sim)"""
    out = {}
    if _guard_short(t):
        return None
    x0, y0, yaw0 = ref_pose(t, x, y, yaw)
    ux, uy = math.cos(yaw0), math.sin(yaw0)
    vx_, vy_, v = speed_profile(t, x, y)
    proj = vx_ * ux + vy_ * uy
    lat = ux * (y - y0) - uy * (x - x0)      # 左正
    yu, _ = yaw_rate_profile(t, yaw)
    dist = path_length(x, y)

    mw = t >= t_win0
    v_act = float(np.mean(v[mw])) if np.any(mw) else float(np.mean(v))
    v_along = float(np.mean(proj[mw])) if np.any(mw) else float(np.mean(proj))
    v_peak = float(np.max(v[mw])) if np.any(mw) else float(np.max(v))

    lat_abs = np.abs(lat)
    yaw_drift_deg = math.degrees(float(yu[-1] - yu[0]))
    out.update({
        "v_act": v_act,
        "v_along": v_along,
        "v_peak": v_peak,
        "eta": (v_act / cmd_vx) if cmd_vx > 1e-6 else None,
        "lat_max_m": float(np.max(lat_abs)),
        "lat_rms_m": float(np.sqrt(np.mean(lat_abs ** 2))),
        "heading_drift_deg": yaw_drift_deg,
        "heading_rate_deg_per_m": abs(yaw_drift_deg) / dist if dist > 1e-6 else None,
        "lateral_ratio": float(np.max(lat_abs)) / dist if dist > 1e-6 else None,
        "dist_m": dist,
    })
    curves = {
        "t": t, "v": v, "lat": lat, "yaw_deg": np.degrees(yu),
    }
    return out, curves


def metrics_turn(t, x, y, yaw, t_win0, cmd_wz):
    """原地转弯段指标"""
    out = {}
    if _guard_short(t):
        return None
    x0, y0, yaw0 = ref_pose(t, x, y, yaw)
    yu, wz = yaw_rate_profile(t, yaw)
    d = np.hypot(x - x0, y - y0)

    mw = t >= t_win0
    wz_act = float(np.mean(wz[mw])) if np.any(mw) else float(np.mean(wz))
    wz_peak = float(np.max(np.abs(wz[mw]))) if np.any(mw) else float(np.max(np.abs(wz)))
    yaw_total = float(yu[-1] - yu[0])       # rad, 有符号
    d_max = float(np.max(d))
    out.update({
        "wz_act": wz_act,
        "wz_peak": wz_peak,
        "eta": (wz_act / cmd_wz) if abs(cmd_wz) > 1e-6 else None,
        "yaw_total_deg": math.degrees(yaw_total),
        "d_max_m": d_max,
        "d_end_m": float(d[-1]),
        "drift_per_rad": d_max / abs(yaw_total) if abs(yaw_total) > 1e-6 else None,
    })
    curves = {"t": t, "wz": wz, "d": d, "yaw_deg": np.degrees(yu)}
    return out, curves


def metrics_arc(t, x, y, yaw, t_win0, cmd_vx, cmd_wz):
    """弧线段指标"""
    out = {}
    if _guard_short(t):
        return None
    x0, y0, yaw0 = ref_pose(t, x, y, yaw)
    _, _, v = speed_profile(t, x, y)
    yu, wz = yaw_rate_profile(t, yaw)

    mw = t >= t_win0
    v_act = float(np.mean(v[mw])) if np.any(mw) else float(np.mean(v))
    wz_act = float(np.mean(wz[mw])) if np.any(mw) else float(np.mean(wz))
    r_cmd = abs(cmd_vx / cmd_wz) if abs(cmd_wz) > 1e-6 else None
    r_act = (v_act / abs(wz_act)) if abs(wz_act) > 1e-6 else None

    fit = kasa_circle_fit(x, y)
    fit_out = {"r_fit": None, "resid_rms_m": None, "resid_max_m": None}
    if fit is not None:
        cx, cy, r_fit, res_rms, res = fit
        fit_out = {
            "r_fit": r_fit,
            "resid_rms_m": res_rms,
            "resid_max_m": float(np.max(np.abs(res))),
        }
    out.update({
        "v_act": v_act,
        "wz_act": wz_act,
        "eta_v": (v_act / cmd_vx) if cmd_vx > 1e-6 else None,
        "eta_w": (wz_act / cmd_wz) if abs(cmd_wz) > 1e-6 else None,
        "r_cmd_m": r_cmd,
        "r_act_m": r_act,
        "r_fit": fit_out["r_fit"],
        "radius_err_pct": (abs(fit_out["r_fit"] - r_cmd) / r_cmd * 100.0)
        if (fit_out["r_fit"] is not None and r_cmd) else None,
        "resid_rms_m": fit_out["resid_rms_m"],
        "resid_max_m": fit_out["resid_max_m"],
        "yaw_total_deg": math.degrees(float(yu[-1] - yu[0])),
        "dir_ok": bool(np.sign(wz_act) == np.sign(cmd_wz)) if abs(wz_act) > 1e-9 else None,
    })
    curves = {"t": t, "v": v, "wz": wz, "x": x, "y": y}
    return out, curves


def metrics_stop(t, x, y, t0_sim, stop_wait=3.0):
    """停止段指标: t0_sim = 零指令时刻 (sim); 数组需覆盖 [t0-1.0, t0+stop_wait+0.5]"""
    out = {
        "v0": None, "t_stop_s": None, "decel_mean": None, "decel_peak": None,
        "decel_p95": None, "stop_dist_m": None, "fully_stopped": False,
    }
    if _guard_short(t):
        return out
    _, _, v = speed_profile(t, x, y)

    m_pre = (t >= t0_sim - 0.6) & (t <= t0_sim - 0.1)
    if np.any(m_pre):
        out["v0"] = float(np.mean(v[m_pre]))

    # 停止时刻: v < 阈值 且持续 STOP_SUSTAIN_S
    dt_med = float(np.median(np.diff(t))) if len(t) > 2 else 0.001
    k = max(3, int(STOP_SUSTAIN_S / max(dt_med, 1e-6)))
    below = v < STOP_V_THRESH
    t_stop = None
    idx0 = np.searchsorted(t, t0_sim)
    for i in range(idx0, max(idx0, len(t) - k)):
        if np.all(below[i:i + k]):
            t_stop = float(t[i])
            break
    if t_stop is None and np.any(t >= t0_sim):
        m_tail = t >= t0_sim
        if np.all(below[m_tail]):
            t_stop = float(t[np.argmax(t >= t0_sim)])
    if t_stop is not None:
        out["t_stop_s"] = t_stop - t0_sim
        out["fully_stopped"] = True
        decel = -np.gradient(v, t)
        m_dec = (t >= t0_sim) & (t <= t_stop)
        if np.any(m_dec):
            dd = decel[m_dec]
            out["decel_peak"] = float(np.max(dd))
            out["decel_p95"] = float(np.percentile(dd, 95))
        if out["v0"]:
            out["decel_mean"] = out["v0"] / max(out["t_stop_s"], 1e-6)
        m_sd = (t >= t0_sim) & (t <= t_stop)
        out["stop_dist_m"] = path_length(x[m_sd], y[m_sd])
    return out


# ═══════════════════════════════════════════════════════════
#  本地自检 (合成数据, 无需 ROS)
# ═══════════════════════════════════════════════════════════

def _selftest():
    print("═══ loco_perf_test.py 指标核心自检 (合成数据) ═══")
    ok_all = True
    dt = 0.001

    def check(name, cond, val=""):
        nonlocal ok_all
        ok_all &= bool(cond)
        print(f"  {'✅' if cond else '❌'} {name}  {val}")

    # 1) 完美直线
    t = np.arange(0, 4.0, dt)
    x, y, yaw = 0.5 * t, np.zeros_like(t), np.zeros_like(t)
    r = metrics_straight(t, x, y, yaw, t_win0=t[-1] - 3.0, cmd_vx=0.5)
    m = r[0]
    check("直线-完美: 达成率≈1", abs(m["eta"] - 1.0) < 0.02, f"eta={m['eta']:.3f}")
    check("直线-完美: 横向≈0", m["lat_max_m"] < 0.005, f"lat_max={m['lat_max_m']:.4f}")
    check("直线-完美: 漂移≈0", abs(m["heading_rate_deg_per_m"]) < 0.05,
          f"rate={m['heading_rate_deg_per_m']:.4f}°/m")

    # 2) 航向漂移 2°/s (弯曲)
    yaw_d = np.radians(2.0) * t
    vx_d = 0.5 * np.cos(yaw_d)
    vy_d = 0.5 * np.sin(yaw_d)
    x_d = np.cumsum(vx_d) * dt
    y_d = np.cumsum(vy_d) * dt
    r = metrics_straight(t, x_d, y_d, yaw_d, t_win0=t[-1] - 3.0, cmd_vx=0.5)
    m = r[0]
    # 期望: 航向漂移率 ≈ 2°/s / 0.5m/s = 4°/m
    check("直线-漂移: 航向漂移率≈4°/m", abs(m["heading_rate_deg_per_m"] - 4.0) < 0.5,
          f"rate={m['heading_rate_deg_per_m']:.2f}°/m")
    check("直线-漂移: 横向偏差>0.05m", m["lat_max_m"] > 0.05, f"lat_max={m['lat_max_m']:.3f}")

    # 3) 原地转弯-完美 (位置不动)
    wz = 0.5
    yaw_t = wz * t
    x_t, y_t = np.zeros_like(t), np.zeros_like(t)
    r = metrics_turn(t, x_t, y_t, yaw_t, t_win0=t[-1] - 3.0, cmd_wz=wz)
    m = r[0]
    check("转弯-完美: 达成率≈1", abs(m["eta"] - 1.0) < 0.02, f"eta={m['eta']:.3f}")
    check("转弯-完美: 位移≈0", m["d_max_m"] < 0.005, f"d_max={m['d_max_m']:.4f}")
    check("转弯-完美: 转过角度≈114.6°", abs(m["yaw_total_deg"] - math.degrees(wz * 4.0)) < 1.0,
          f"yaw={m['yaw_total_deg']:.1f}°")

    # 4) 原地转弯-带 1cm/s 漂移
    x_t2 = 0.01 * t
    r = metrics_turn(t, x_t2, np.zeros_like(t), yaw_t, t_win0=t[-1] - 3.0, cmd_wz=wz)
    m = r[0]
    dr = m["d_max_m"] / abs(math.radians(m["yaw_total_deg"]))
    check("转弯-漂移: d_max≈0.04m", abs(m["d_max_m"] - 0.04) < 0.005, f"d_max={m['d_max_m']:.4f}")
    check("转弯-漂移: m/rad > 0", dr > 0.005, f"{dr:.4f} m/rad")

    # 5) 弧线-完美 R=1m
    v, w = 0.4, 0.4
    th = w * t
    x_a, y_a = 1.0 * np.sin(th), 1.0 * (1 - np.cos(th))
    r = metrics_arc(t, x_a, y_a, th, t_win0=t[-1] - 3.0, cmd_vx=v, cmd_wz=w)
    m = r[0]
    check("弧线-完美: R拟合≈1m", abs(m["r_fit"] - 1.0) < 0.05, f"r_fit={m['r_fit']:.3f}")
    check("弧线-完美: 半径误差<2%", m["radius_err_pct"] < 2.0, f"err={m['radius_err_pct']:.2f}%")
    check("弧线-完美: 残差≈0", m["resid_rms_m"] < 0.01, f"rms={m['resid_rms_m']:.4f}")

    # 6) 弧线-半径偏差 20%
    x_a2, y_a2 = 1.2 * np.sin(th), 1.2 * (1 - np.cos(th))
    r = metrics_arc(t, x_a2, y_a2, th, t_win0=t[-1] - 3.0, cmd_vx=v, cmd_wz=w)
    m = r[0]
    check("弧线-偏差: 半径误差≈20%", abs(m["radius_err_pct"] - 20.0) < 2.0,
          f"err={m['radius_err_pct']:.1f}%")

    # 7) 停止: 0.4 m/s, 线性减速 0.5s
    t2 = np.arange(0, 6.0, dt)
    vprof = np.where(t2 < 3.0, 0.4,
                     np.where(t2 < 3.5, 0.4 * (1 - (t2 - 3.0) / 0.5), 0.0))
    x_s = np.cumsum(vprof) * dt
    r = metrics_stop(t2, x_s, np.zeros_like(t2), t0_sim=3.0, stop_wait=3.0)
    # 理论停止时间(首次低于 0.05 m/s): (1 - 0.05/0.4) * 0.5 = 0.4375s
    t_exp = (1 - STOP_V_THRESH / 0.4) * 0.5
    check("停止: 停止时间≈0.4375s", r["t_stop_s"] is not None and abs(r["t_stop_s"] - t_exp) < 0.03,
          f"t_stop={r['t_stop_s']}")
    check("停止: 峰值减速≈0.8", abs(r["decel_peak"] - 0.8) < 0.08, f"peak={r['decel_peak']:.3f}")
    check("停止: v0≈0.4", abs(r["v0"] - 0.4) < 0.01, f"v0={r['v0']:.3f}")
    check("停止: 停止距离>0", (r["stop_dist_m"] or 0) > 0.05, f"dist={r['stop_dist_m']:.3f}")

    print("═══ 自检结果:", "全部通过 ✅" if ok_all else "存在失败 ❌", "═══")
    return 0 if ok_all else 1


# ═══════════════════════════════════════════════════════════
#  ROS 节点与测试执行
# ═══════════════════════════════════════════════════════════

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import Float32, Float64MultiArray
    from geometry_msgs.msg import Twist
    HAVE_ROS = True
except Exception:  # 无 ROS 环境 (如开发机) 仅供 --selftest
    HAVE_ROS = False


def _gt_arrays(node):
    if not node.gt:
        return None
    return np.asarray(node.gt, dtype=float)  # [wall, sim, x, y, z, roll, pitch, yaw, rtf, coll, cum]


def _slice_gt(node, t0, t1):
    a = _gt_arrays(node)
    if a is None:
        return None
    m = (a[:, 1] >= t0) & (a[:, 1] <= t1)
    return a[m]


def _wall_to_sim(node, wall_t):
    """wall 时刻 -> sim 时刻 (局部线性回归, 抵消传输延迟)"""
    a = _gt_arrays(node)
    if a is None or len(a) < 10:
        return None
    m = np.abs(a[:, 0] - wall_t) < 2.5
    if m.sum() < 10:
        m = np.ones(len(a), dtype=bool)
    coef = np.polyfit(a[m, 0], a[m, 1], 1)
    return float(np.polyval(coef, wall_t))


def _downsample(arr, max_pts=150):
    arr = np.asarray(arr, float)
    n = len(arr)
    if n <= max_pts:
        return [round(float(v), 4) for v in arr]
    idx = np.linspace(0, n - 1, max_pts).astype(int)
    return [round(float(arr[i]), 4) for i in idx]


class LocoPerfNode(Node if HAVE_ROS else object):
    def __init__(self, args):
        super().__init__("loco_perf_test")
        self.args = args
        self.pub_dt = 1.0 / max(args.publish_hz, 5.0)
        self.fall_z = args.fall_z
        self.fall_deg = args.fall_deg

        self.gt = []            # (wall, sim, x, y, z, roll, pitch, yaw, rtf, coll, cum)
        self.cmd_log = []       # (wall, vx, vy, wz)
        self.fall = None        # 首次摔倒信息
        self.last_sim = None
        self.last_gt_wall = None
        self._pub_last = 0.0
        self._cur_cmd = None
        self.wall_cap = args.timeout_wall

        self.cmd_pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.walk_pub = self.create_publisher(Float32, "/walk_mode", 10)
        self.stand_pub = self.create_publisher(Float32, "/stand_mode", 10)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1000,
        )
        self.create_subscription(Float64MultiArray, GT_TOPIC, self._gt_cb, sensor_qos)

    # ---------- 回调 ----------
    def _gt_cb(self, msg):
        d = list(msg.data)
        if len(d) < 10:
            return
        wall = time.monotonic()
        self.gt.append((wall, d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7], d[8], d[9]))
        self.last_sim = d[0]
        self.last_gt_wall = wall
        if self.fall is None:
            z, roll, pitch = d[3], d[4], d[5]
            thr = math.radians(self.fall_deg)
            reason = None
            if z < self.fall_z:
                reason = f"z={z:.3f}<{self.fall_z}"
            elif abs(roll) > thr:
                reason = f"|roll|={math.degrees(abs(roll)):.1f}°>{self.fall_deg}°"
            elif abs(pitch) > thr:
                reason = f"|pitch|={math.degrees(abs(pitch)):.1f}°>{self.fall_deg}°"
            if reason:
                self.fall = {"sim": d[0], "wall": wall, "reason": reason,
                             "z": z, "roll_deg": math.degrees(roll),
                             "pitch_deg": math.degrees(pitch)}

    # ---------- 基础操作 ----------
    def sim_now(self, timeout_wall=30.0):
        t0 = time.monotonic()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.last_sim is not None and (time.monotonic() - self.last_gt_wall) < GT_STALE_S:
                return self.last_sim
            if time.monotonic() - t0 > timeout_wall:
                raise RuntimeError(f"等待 {GT_TOPIC} 超时 ({timeout_wall}s), 仿真未启动或未启用 GT")
        raise RuntimeError("rclpy 已关闭")

    def publish_cmd(self, vx, vy, wz, force=False):
        now = time.monotonic()
        cmd = (round(vx, 6), round(vy, 6), round(wz, 6))
        if (not force) and self._cur_cmd == cmd and (now - self._pub_last) < self.pub_dt:
            return None
        tw = Twist()
        tw.linear.x, tw.linear.y, tw.angular.z = float(vx), float(vy), float(wz)
        self.cmd_pub.publish(tw)
        self._pub_last = now
        self._cur_cmd = cmd
        self.cmd_log.append((now, float(vx), float(vy), float(wz)))
        return now

    def drive(self, vx, vy, wz, dur_sim=None, abort_on_fall=True):
        """保持 (vx,vy,wz) 直到仿真时间前进 dur_sim; 返回 {t0,t1,pub_wall,fell}"""
        pub_wall = self.publish_cmd(vx, vy, wz, force=True)
        t0 = self.last_sim if self.last_sim is not None else self.sim_now()
        t_start = time.monotonic()
        fell = False
        while True:
            self.publish_cmd(vx, vy, wz)
            rclpy.spin_once(self, timeout_sec=0.004)
            if self.fall is not None and abort_on_fall:
                fell = True
                break
            now = time.monotonic()
            if self.last_gt_wall is None or (now - self.last_gt_wall) > GT_STALE_S:
                raise RuntimeError("ground truth 断流, 中止")
            if now - t_start > self.wall_cap:
                raise RuntimeError(f"drive 超过 wall 上限 {self.wall_cap}s, 中止")
            if dur_sim is not None and (self.last_sim - t0) >= dur_sim:
                break
        return {"t0": float(t0), "t1": float(self.last_sim), "pub_wall": pub_wall, "fell": bool(fell)}

    def pulse(self, pub, secs=1.2, hz=5.0):
        n = max(1, int(secs * hz))
        for _ in range(n):
            m = Float32()
            m.data = 0.0
            pub.publish(m)
            rclpy.spin_once(self, timeout_sec=1.0 / hz)

    def enter_walk(self, settle=2.0, skip=False):
        if not skip:
            print("  [setup] 发布 /stand_mode (1.2s) ...")
            self.pulse(self.stand_pub, secs=1.2)
            self.drive(0.0, 0.0, 0.0, dur_sim=1.0)
            print("  [setup] 发布 /walk_mode (2s, 控制端有 1s 节流) ...")
            self.pulse(self.walk_pub, secs=2.0)
        print(f"  [setup] 进入 walk_leg, 稳定 {settle}s ...")
        self.drive(0.0, 0.0, 0.0, dur_sim=settle)

    def armed_check(self):
        """小速度脉冲, 验证 walk 模式确已激活; 返回实测速度"""
        self.drive(0.2, 0.0, 0.0, dur_sim=1.8)
        t1 = self.last_sim
        a = _slice_gt(self, t1 - 0.8, t1)
        v_mean = None
        if a is not None and len(a) > 8:
            _, _, v = speed_profile(a[:, 1], a[:, 2], a[:, 3])
            v_mean = float(np.mean(v))
        self.drive(0.0, 0.0, 0.0, dur_sim=0.8)
        return v_mean

    def finish(self, to_stand=True):
        try:
            self.drive(0.0, 0.0, 0.0, dur_sim=0.6, abort_on_fall=False)
        except Exception:
            pass
        if to_stand:
            self.pulse(self.stand_pub, secs=1.2)

    def gt_meta(self):
        a = _gt_arrays(self)
        if a is None or len(a) < 2:
            return {"samples": 0}
        dur = float(a[-1, 1] - a[0, 1])
        rtfs = a[:, 8]
        rtfs = rtfs[rtfs > 0]
        return {
            "samples": int(len(a)),
            "gt_rate_hz": round(len(a) / dur, 1) if dur > 1e-6 else None,
            "sim_duration_s": round(dur, 2),
            "rtf_mean": round(float(np.mean(rtfs)), 3) if len(rtfs) else None,
        }


# ---------- 单项测试流程 ----------

def _collect_levels(node, levels, drive_fn, args, tag=""):
    """通用档位扫描: 每档保持 hold 秒, 摔倒即停; 返回 records"""
    records = []
    for lv in levels:
        rec = drive_fn(lv)
        rec["cmd"] = lv
        records.append(rec)
        state = "摔倒!" if rec["fell"] else "ok"
        print(f"  [{tag}] level={lv}  sim_t={rec['t0']:.1f}->{rec['t1']:.1f}  {state}")
        if rec["fell"]:
            break
    return records


def run_straight(node, args):
    levels = [float(s) for s in args.levels.split(",")]
    records = []
    status = "ok"
    try:
        node.enter_walk(settle=args.settle, skip=args.skip_mode_switch)
        if not args.no_armed_check:
            v = node.armed_check()
            print(f"  [check] walk 激活检查: v={v}")
            if v is None or v < 0.05:
                print("  [check] ❌ walk 模式未激活或机器人未运动, 终止 (可用 --no-armed-check 跳过)")
                return 3, None
        node.drive(0.0, 0.0, 0.0, dur_sim=args.warmup)
        records = _collect_levels(node, levels,
                                  lambda lv: node.drive(lv, 0.0, 0.0, dur_sim=args.hold),
                                  args, tag="straight")
    except KeyboardInterrupt:
        status = "interrupted"
        print("  [warn] 用户中断, 保存已采集数据")
    except Exception as e:
        status = "aborted"
        print(f"  [error] 中止: {e}")
    finally:
        node.finish(to_stand=not args.no_stand_finish)

    result = _analyze_straight(node, records, levels, args)
    result["status"] = ("fall" if any(r["fell"] for r in records) else status)
    _save_and_report(result, args)
    return (2 if result["status"] == "fall" else
            (3 if result["status"] in ("interrupted", "aborted") else 0)), result


def run_turn(node, args):
    levels = [float(s) for s in args.levels.split(",")]
    records = []
    status = "ok"
    try:
        node.enter_walk(settle=args.settle, skip=args.skip_mode_switch)
        if not args.no_armed_check:
            v = node.armed_check()
            print(f"  [check] walk 激活检查: v={v}")
            if v is None or v < 0.05:
                print("  [check] ❌ walk 模式未激活或机器人未运动, 终止")
                return 3, None
        node.drive(0.0, 0.0, 0.0, dur_sim=args.warmup)
        records = _collect_levels(node, levels,
                                  lambda lv: node.drive(0.0, 0.0, lv, dur_sim=args.hold),
                                  args, tag="turn")
    except KeyboardInterrupt:
        status = "interrupted"
        print("  [warn] 用户中断, 保存已采集数据")
    except Exception as e:
        status = "aborted"
        print(f"  [error] 中止: {e}")
    finally:
        node.finish(to_stand=not args.no_stand_finish)

    result = _analyze_turn(node, records, levels, args)
    result["status"] = ("fall" if any(r["fell"] for r in records) else status)
    _save_and_report(result, args)
    return (2 if result["status"] == "fall" else
            (3 if result["status"] in ("interrupted", "aborted") else 0)), result


def run_arc(node, args):
    levels = [[float(a), float(b)] for p in args.levels.split(",") for a, b in [p.split(":")]]
    records = []
    status = "ok"
    try:
        node.enter_walk(settle=args.settle, skip=args.skip_mode_switch)
        if not args.no_armed_check:
            v = node.armed_check()
            print(f"  [check] walk 激活检查: v={v}")
            if v is None or v < 0.05:
                print("  [check] ❌ walk 模式未激活或机器人未运动, 终止")
                return 3, None
        node.drive(0.0, 0.0, 0.0, dur_sim=args.warmup)
        records = _collect_levels(node, levels,
                                  lambda lv: node.drive(lv[0], 0.0, lv[1], dur_sim=args.hold),
                                  args, tag="arc")
    except KeyboardInterrupt:
        status = "interrupted"
        print("  [warn] 用户中断, 保存已采集数据")
    except Exception as e:
        status = "aborted"
        print(f"  [error] 中止: {e}")
    finally:
        node.finish(to_stand=not args.no_stand_finish)

    result = _analyze_arc(node, records, levels, args)
    result["status"] = ("fall" if any(r["fell"] for r in records) else status)
    _save_and_report(result, args)
    return (2 if result["status"] == "fall" else
            (3 if result["status"] in ("interrupted", "aborted") else 0)), result


def run_stop(node, args):
    speeds = [float(s) for s in args.stop_speeds.split(",")]
    records = []
    status = "ok"
    try:
        node.enter_walk(settle=args.settle, skip=args.skip_mode_switch)
        if not args.no_armed_check:
            v = node.armed_check()
            print(f"  [check] walk 激活检查: v={v}")
            if v is None or v < 0.05:
                print("  [check] ❌ walk 模式未激活或机器人未运动, 终止")
                return 3, None
        node.drive(0.0, 0.0, 0.0, dur_sim=args.warmup)
        stop_all = False
        for spd in speeds:
            if stop_all:
                break
            for r in range(args.stop_reps):
                rec_a = node.drive(spd, 0.0, 0.0, dur_sim=args.stop_approach)
                if rec_a["fell"]:
                    records.append({"speed": spd, "rep": r, "approach": rec_a, "stop": None})
                    stop_all = True
                    break
                rec_s = node.drive(0.0, 0.0, 0.0, dur_sim=args.stop_wait)
                records.append({"speed": spd, "rep": r, "approach": rec_a, "stop": rec_s})
                state = "摔倒!" if rec_s["fell"] else "ok"
                print(f"  [stop] speed={spd:.2f} rep={r}  "
                      f"approach_t={rec_a['t0']:.1f}->{rec_a['t1']:.1f}  {state}")
                if rec_s["fell"]:
                    stop_all = True
                    break
    except KeyboardInterrupt:
        status = "interrupted"
        print("  [warn] 用户中断, 保存已采集数据")
    except Exception as e:
        status = "aborted"
        print(f"  [error] 中止: {e}")
    finally:
        node.finish(to_stand=not args.no_stand_finish)

    result = _analyze_stop(node, records, speeds, args)
    fell_any = any((r.get("stop") or {}).get("fell") or
                   ((r.get("approach") or {}).get("fell") if "approach" in r else False)
                   for r in records)
    result["status"] = ("fall" if fell_any else status)
    _save_and_report(result, args)
    return (2 if result["status"] == "fall" else
            (3 if result["status"] in ("interrupted", "aborted") else 0)), result


# ---------- 结果组装 ----------

def _level_metrics_straight(node, rec, args):
    t1_eff = rec["t1"]
    if rec["fell"] and node.fall is not None:
        t1_eff = min(t1_eff, node.fall["sim"] - 0.1)
    a = _slice_gt(node, rec["t0"] + 0.5, t1_eff)
    if a is None or len(a) < 20:
        return None
    t_win0 = max(rec["t0"] + 0.5, t1_eff - args.window)
    r = metrics_straight(a[:, 1], a[:, 2], a[:, 3], a[:, 7], t_win0, rec["cmd"])
    if r is None:
        return None
    m, curves = r
    m["fallen"] = rec["fell"]
    m["curves"] = {
        "t": _downsample(curves["t"] - rec["t0"]),
        "v": _downsample(curves["v"]),
        "lat": _downsample(curves["lat"]),
        "yaw_deg": _downsample(curves["yaw_deg"]),
    }
    return m


def _analyze_straight(node, records, levels, args):
    lv_out = []
    for i, rec in enumerate(records):
        m = _level_metrics_straight(node, rec, args)
        entry = {"cmd_vx": rec["cmd"], "t0": rec["t0"], "t1": rec["t1"], "fallen": rec["fell"]}
        if m:
            entry.update(m)
        lv_out.append(entry)

    max_nf = None
    max_tracked = None
    for e in lv_out:
        if not e["fallen"]:
            max_nf = e["cmd_vx"]
        if not e["fallen"] and e.get("eta") is not None and e["eta"] >= 0.5:
            max_tracked = e["cmd_vx"]
    cap_reached = bool(lv_out) and all(not e["fallen"] for e in lv_out)
    best = next((e for e in lv_out if e["cmd_vx"] == max_nf), None) if max_nf is not None else None

    straight_ok = None
    if best and best.get("heading_rate_deg_per_m") is not None:
        lr = best.get("lateral_ratio")
        straight_ok = (best["heading_rate_deg_per_m"] <= V_STRAIGHT_HEADING_RATE_MAX and
                       lr is not None and lr <= V_STRAIGHT_LATERAL_RATIO_MAX)

    fall_at = next((e["cmd_vx"] for e in lv_out if e["fallen"]), None)
    summary = {
        "max_no_fall_vx": max_nf,
        "max_no_fall_eta": (best or {}).get("eta"),
        "max_tracked_vx": max_tracked,
        "cap_reached": cap_reached,
        "fall_at_vx": fall_at,
        "straight_ok": straight_ok,
        "fall_reason": node.fall["reason"] if node.fall else None,
    }
    result = {
        "test": "straight",
        "levels": lv_out,
        "summary": summary,
        "meta": node.gt_meta(),
    }
    _print_straight_summary(summary, best)
    return result


def _analyze_turn(node, records, levels, args):
    lv_out = []
    for rec in records:
        t1_eff = rec["t1"]
        if rec["fell"] and node.fall is not None:
            t1_eff = min(t1_eff, node.fall["sim"] - 0.1)
        a = _slice_gt(node, rec["t0"] + 0.5, t1_eff)
        entry = {"cmd_wz": rec["cmd"], "t0": rec["t0"], "t1": rec["t1"], "fallen": rec["fell"]}
        if a is not None and len(a) >= 20:
            t_win0 = max(rec["t0"] + 0.5, t1_eff - args.window)
            r = metrics_turn(a[:, 1], a[:, 2], a[:, 3], a[:, 7], t_win0, rec["cmd"])
            if r is not None:
                m, curves = r
                entry.update(m)
                entry["curves"] = {
                    "t": _downsample(curves["t"] - rec["t0"]),
                    "wz": _downsample(curves["wz"]),
                    "d": _downsample(curves["d"]),
                }
        lv_out.append(entry)

    max_nf = None
    max_tracked = None
    for e in lv_out:
        if not e["fallen"]:
            max_nf = e["cmd_wz"]
        if not e["fallen"] and e.get("eta") is not None and e["eta"] >= 0.5:
            max_tracked = e["cmd_wz"]
    cap_reached = bool(lv_out) and all(not e["fallen"] for e in lv_out)
    best = next((e for e in lv_out if e["cmd_wz"] == max_nf), None) if max_nf is not None else None

    inplace_ok = None
    if best and best.get("d_max_m") is not None:
        dpr = best.get("drift_per_rad")
        inplace_ok = (best["d_max_m"] <= V_TURN_INPLACE_DMAX_MAX and
                      dpr is not None and dpr <= V_TURN_DRIFT_PER_RAD_MAX)

    summary = {
        "max_no_fall_wz": max_nf,
        "max_no_fall_eta": (best or {}).get("eta"),
        "max_tracked_wz": max_tracked,
        "cap_reached": cap_reached,
        "fall_at_wz": next((e["cmd_wz"] for e in lv_out if e["fallen"]), None),
        "inplace_ok": inplace_ok,
        "fall_reason": node.fall["reason"] if node.fall else None,
    }
    result = {"test": "turn", "levels": lv_out, "summary": summary, "meta": node.gt_meta()}
    _print_turn_summary(summary, best)
    return result


def _analyze_arc(node, records, levels, args):
    lv_out = []
    for rec in records:
        t1_eff = rec["t1"]
        if rec["fell"] and node.fall is not None:
            t1_eff = min(t1_eff, node.fall["sim"] - 0.1)
        a = _slice_gt(node, rec["t0"] + 0.5, t1_eff)
        entry = {"cmd_vx": rec["cmd"][0], "cmd_wz": rec["cmd"][1],
                 "t0": rec["t0"], "t1": rec["t1"], "fallen": rec["fell"]}
        if a is not None and len(a) >= 20:
            t_win0 = max(rec["t0"] + 0.5, t1_eff - args.window)
            r = metrics_arc(a[:, 1], a[:, 2], a[:, 3], a[:, 7], t_win0,
                            rec["cmd"][0], rec["cmd"][1])
            if r is not None:
                m, curves = r
                entry.update(m)
                entry["curves"] = {
                    "t": _downsample(curves["t"] - rec["t0"]),
                    "v": _downsample(curves["v"]),
                    "wz": _downsample(curves["wz"]),
                    "x": _downsample(curves["x"]),
                    "y": _downsample(curves["y"]),
                }
        lv_out.append(entry)

    max_nf = None
    max_tracked = None
    for e in lv_out:
        if not e["fallen"]:
            max_nf = (e["cmd_vx"], e["cmd_wz"])
        if (not e["fallen"] and e.get("eta_v") is not None and e["eta_v"] >= 0.5
                and e.get("eta_w") is not None and e["eta_w"] >= 0.5):
            max_tracked = (e["cmd_vx"], e["cmd_wz"])
    cap_reached = bool(lv_out) and all(not e["fallen"] for e in lv_out)
    best = next((e for e in lv_out if (e["cmd_vx"], e["cmd_wz"]) == max_nf), None) if max_nf else None

    arc_ok = None
    if best and best.get("radius_err_pct") is not None:
        rr = best.get("resid_rms_m")
        arc_ok = (best["radius_err_pct"] <= V_ARC_RADIUS_ERR_MAX * 100.0 and
                  rr is not None and rr <= V_ARC_RESID_RMS_MAX)

    summary = {
        "max_no_fall_vw": max_nf,
        "max_no_fall_eta_v": (best or {}).get("eta_v"),
        "max_no_fall_eta_w": (best or {}).get("eta_w"),
        "max_tracked_vw": max_tracked,
        "cap_reached": cap_reached,
        "fall_at_vw": next(((e["cmd_vx"], e["cmd_wz"]) for e in lv_out if e["fallen"]), None),
        "arc_ok": arc_ok,
        "fall_reason": node.fall["reason"] if node.fall else None,
    }
    result = {"test": "arc", "levels": lv_out, "summary": summary, "meta": node.gt_meta()}
    _print_arc_summary(summary, best)
    return result


def _analyze_stop(node, records, speeds, args):
    reps_out = []
    for rec in records:
        if rec.get("stop") is None:
            # 提速段即摔倒, 无停止数据
            reps_out.append({"speed_cmd": rec["speed"], "rep": rec["rep"], "approach_fell": True})
            continue
        t_zero_wall = rec["stop"]["pub_wall"]
        t0_fit = _wall_to_sim(node, t_zero_wall)
        t0_sim = t0_fit if t0_fit is not None else rec["stop"]["t0"]
        m = None
        a = _slice_gt(node, t0_sim - 1.0, t0_sim + args.stop_wait + 0.5)
        if a is not None and len(a) >= 20:
            m = metrics_stop(a[:, 1], a[:, 2], a[:, 3], t0_sim, stop_wait=args.stop_wait)
        entry = {"speed_cmd": rec["speed"], "rep": rec["rep"],
                 "t_zero_sim": round(float(t0_sim), 3),
                 "t_zero_sim_latest_gt": round(float(rec["stop"]["t0"]), 3)}
        if m:
            entry.update(m)
        reps_out.append(entry)

    def _stat(key, fn):
        vals = [e[key] for e in reps_out if e.get(key) is not None]
        return round(fn(vals), 3) if vals else None

    summary = {
        "t_stop_mean": _stat("t_stop_s", lambda v: float(np.mean(v))),
        "t_stop_max": _stat("t_stop_s", lambda v: float(np.max(v))),
        "decel_peak_max": _stat("decel_peak", lambda v: float(np.max(v))),
        "decel_peak_mean": _stat("decel_peak", lambda v: float(np.mean(v))),
        "stop_dist_mean": _stat("stop_dist_m", lambda v: float(np.mean(v))),
        "reps_ok": sum(1 for e in reps_out if e.get("fully_stopped")),
        "reps_total": len(reps_out),
    }
    result = {"test": "stop", "reps": reps_out, "summary": summary, "meta": node.gt_meta()}
    print("  [summary] 停止: "
          f"t_stop mean={summary['t_stop_mean']}s max={summary['t_stop_max']}s, "
          f"decel_peak max={summary['decel_peak_max']} m/s², "
          f"停止距离 mean={summary['stop_dist_mean']} m")
    return result


# ---------- 控制台摘要 ----------

def _print_straight_summary(s, best):
    print("  [summary] 直线: 最大速度(不摔倒)="
          f"{s['max_no_fall_vx'] if s['max_no_fall_vx'] is not None else '无'}"
          f"{' m/s (已达上限, 未摔倒)' if s['cap_reached'] else ''}")
    if best:
        print(f"            @该档: 达成率={best.get('eta')}, "
              f"航向漂移率={best.get('heading_rate_deg_per_m')}°/m, "
              f"偏移率={best.get('lateral_ratio')}")
    if s["fall_at_vx"] is not None:
        print(f"            在 {s['fall_at_vx']} m/s 摔倒: {s['fall_reason']}")


def _print_turn_summary(s, best):
    print("  [summary] 转弯: 最大角速度(不摔倒)="
          f"{s['max_no_fall_wz'] if s['max_no_fall_wz'] is not None else '无'}"
          f"{' rad/s (已达上限)' if s['cap_reached'] else ''}")
    if best:
        print(f"            @该档: 达成率={best.get('eta')}, "
              f"原地最大位移={best.get('d_max_m')}m, "
              f"偏移率={best.get('drift_per_rad')} m/rad")


def _print_arc_summary(s, best):
    print(f"  [summary] 弧线: 最大执行速度(不摔倒)={s['max_no_fall_vw']}"
          f"{' (已达上限)' if s['cap_reached'] else ''}")
    if best:
        print(f"            @该档: 达成率 v={best.get('eta_v')} w={best.get('eta_w')}, "
              f"半径误差={best.get('radius_err_pct')}%, "
              f"残差RMS={best.get('resid_rms_m')}m")


# ---------- 落盘 ----------

def _save_and_report(result, args):
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    rdir = os.path.abspath(args.report_dir)
    os.makedirs(rdir, exist_ok=True)
    base = os.path.join(rdir, f"loco_{result['test']}_{ts}")

    with open(base + ".json", "w") as f:
        json.dump(result, f, indent=1, ensure_ascii=False)
    md = _render_md(result, args)
    with open(base + ".md", "w") as f:
        f.write(md)
    print(f"  [report] {base}.json / .md")
    result["_report_base"] = base


def _render_md(r, args):
    L = []
    L.append(f"# 运动控制基础性能测试: {r['test']}")
    L.append("")
    L.append(f"**状态**: {r.get('status') or '—'}  **时间**: {datetime.now().isoformat(timespec='seconds')}")
    m = r.get("meta", {})
    L.append(f"**数据**: GT {m.get('samples')} 帧, {m.get('gt_rate_hz')} Hz, "
             f"RTF mean={m.get('rtf_mean')}")
    L.append("")

    if r["test"] == "straight":
        L.append("## 总览")
        s = r["summary"]
        L.append("")
        L.append("| 指标 | 值 |")
        L.append("|------|-----|")
        L.append(f"| 最大直线速度 (不摔倒) | {s['max_no_fall_vx']} m/s"
                 f"{' (已达扫描上限)' if s['cap_reached'] else ''} |")
        L.append(f"| 该档速度达成率 | {s.get('max_no_fall_eta')} |")
        L.append(f"| 最大跟踪速度 (达成率≥50%) | {s['max_tracked_vx']} m/s |")
        L.append(f"| 是否走直线 (最大档) | {s['straight_ok']} |")
        L.append(f"| 摔倒档位 | {s['fall_at_vx']} {('- ' + str(s['fall_reason'])) if s['fall_reason'] else ''} |")
        L.append("")
        L.append("## 分级明细")
        L.append("")
        L.append("| 指令 vx (m/s) | 实测 v (m/s) | 达成率 | 横向偏差max (m) | 偏移率 | 航向漂移率 (°/m) | 摔倒 |")
        L.append("|---|---|---|---|---|---|---|")
        for e in r["levels"]:
            L.append(f"| {e['cmd_vx']} | {_f(e.get('v_act'))} | {_f(e.get('eta'))} | "
                     f"{_f(e.get('lat_max_m'))} | {_f(e.get('lateral_ratio'))} | "
                     f"{_f(e.get('heading_rate_deg_per_m'))} | {'是' if e['fallen'] else '否'} |")

    elif r["test"] == "turn":
        L.append("## 总览")
        s = r["summary"]
        L.append("")
        L.append("| 指标 | 值 |")
        L.append("|------|-----|")
        L.append(f"| 最大角速度 (不摔倒) | {s['max_no_fall_wz']} rad/s"
                 f"{' (已达扫描上限)' if s['cap_reached'] else ''} |")
        L.append(f"| 该档转速达成率 | {s.get('max_no_fall_eta')} |")
        L.append(f"| 最大跟踪角速度 (达成率≥50%) | {s['max_tracked_wz']} rad/s |")
        L.append(f"| 是否偏移原地 (最大档) | {s['inplace_ok']} |")
        L.append(f"| 摔倒档位 | {s['fall_at_wz']} {('- ' + str(s['fall_reason'])) if s['fall_reason'] else ''} |")
        L.append("")
        L.append("## 分级明细")
        L.append("")
        L.append("| 指令 wz (rad/s) | 实测 wz (rad/s) | 达成率 | 原地最大位移 (m) | 偏移率 (m/rad) | 转过角度 (°) | 摔倒 |")
        L.append("|---|---|---|---|---|---|---|")
        for e in r["levels"]:
            L.append(f"| {e['cmd_wz']} | {_f(e.get('wz_act'))} | {_f(e.get('eta'))} | "
                     f"{_f(e.get('d_max_m'))} | {_f(e.get('drift_per_rad'))} | "
                     f"{_f(e.get('yaw_total_deg'))} | {'是' if e['fallen'] else '否'} |")

    elif r["test"] == "arc":
        L.append("## 总览")
        s = r["summary"]
        L.append("")
        L.append("| 指标 | 值 |")
        L.append("|------|-----|")
        L.append(f"| 最大执行速度 (不摔倒) | v={s['max_no_fall_vw'][0] if s['max_no_fall_vw'] else None} "
                 f"w={s['max_no_fall_vw'][1] if s['max_no_fall_vw'] else None}"
                 f"{' (已达扫描上限)' if s['cap_reached'] else ''} |")
        L.append(f"| 该档达成率 | v={s.get('max_no_fall_eta_v')} w={s.get('max_no_fall_eta_w')} |")
        L.append(f"| 是否圆周跟踪 OK (最大档) | {s['arc_ok']} |")
        L.append(f"| 摔倒档位 | {s['fall_at_vw']} {('- ' + str(s['fall_reason'])) if s['fall_reason'] else ''} |")
        L.append("")
        L.append("## 分级明细")
        L.append("")
        L.append("| 指令 vx | 指令 wz | 实测 v | 达成率v | 达成率w | 拟合半径 R (m) | 半径误差 | 残差RMS (m) | 摔倒 |")
        L.append("|---|---|---|---|---|---|---|---|---|")
        for e in r["levels"]:
            L.append(f"| {e['cmd_vx']} | {e['cmd_wz']} | {_f(e.get('v_act'))} | "
                     f"{_f(e.get('eta_v'))} | {_f(e.get('eta_w'))} | {_f(e.get('r_fit'))} | "
                     f"{_f(e.get('radius_err_pct'))}% | {_f(e.get('resid_rms_m'))} | "
                     f"{'是' if e['fallen'] else '否'} |")

    elif r["test"] == "stop":
        L.append("## 总览")
        s = r["summary"]
        L.append("")
        L.append("| 指标 | 值 |")
        L.append("|------|-----|")
        L.append(f"| 停止时间 (0.4→0) mean/max | {s['t_stop_mean']} / {s['t_stop_max']} s |")
        L.append(f"| 峰值减速度 max/mean | {s['decel_peak_max']} / {s['decel_peak_mean']} m/s² |")
        L.append(f"| 停止距离 mean | {s['stop_dist_mean']} m |")
        L.append(f"| 完整停止次数 | {s['reps_ok']}/{s['reps_total']} |")
        L.append("")
        L.append("## 每次明细")
        L.append("")
        L.append("| 目标速度 (m/s) | # | v0 实测 | 停止时间 (s) | 峰值减速 (m/s²) | 平均减速 (m/s²) | 停止距离 (m) |")
        L.append("|---|---|---|---|---|---|---|")
        for e in r["reps"]:
            L.append(f"| {e['speed_cmd']} | {e['rep']} | {_f(e.get('v0'))} | {_f(e.get('t_stop_s'))} | "
                     f"{_f(e.get('decel_peak'))} | {_f(e.get('decel_mean'))} | {_f(e.get('stop_dist_m'))} |")

    L.append("")
    L.append("## 备注")
    L.append("- 指标口径与判定阈值见 `doc/motion_control_perf_test.md` (阈值初值, 待基线校准)")
    L.append("- 摔倒判据: z<0.35m 或 |roll|>45° 或 |pitch|>45°")
    return "\n".join(L)


def _f(v):
    if v is None:
        return "—"
    if isinstance(v, float):
        return f"{v:.3f}".rstrip("0").rstrip(".")
    return str(v)


# ═══════════════════════════════════════════════════════════
#  主入口
# ═══════════════════════════════════════════════════════════

def build_parser():
    p = argparse.ArgumentParser(description="运动控制(仿真)基础性能测试")
    p.add_argument("--test", choices=["straight", "turn", "arc", "stop"],
                   help="测试项 (单项执行; 多项请用 run_loco_perf.sh 编排)")
    p.add_argument("--levels", type=str, default=None,
                   help="档位序列: straight/turn 为逗号分隔标量; arc 为 vx:wz 逗号分隔")
    p.add_argument("--hold", type=float, default=6.0, help="每档保持时间 (sim s)")
    p.add_argument("--window", type=float, default=3.0, help="稳态窗口长度 (sim s, 取档尾)")
    p.add_argument("--settle", type=float, default=2.0, help="进入 walk 后稳定时间 (sim s)")
    p.add_argument("--warmup", type=float, default=1.2, help="扫描前零速热身 (sim s)")
    p.add_argument("--publish-hz", type=float, default=50.0, help="速度指令发布频率")
    p.add_argument("--report-dir", type=str, default="reports")
    p.add_argument("--fall-z", type=float, default=FALL_Z_M)
    p.add_argument("--fall-deg", type=float, default=FALL_ANGLE_DEG)
    p.add_argument("--stop-speeds", type=str, default="0.4", help="停止测试初速列表 (m/s)")
    p.add_argument("--stop-reps", type=int, default=5, help="每个初速重复次数")
    p.add_argument("--stop-approach", type=float, default=3.5, help="提速保持段时长 (sim s)")
    p.add_argument("--stop-wait", type=float, default=3.5, help="零指令后观察时长 (sim s)")
    p.add_argument("--no-armed-check", action="store_true", help="跳过 walk 模式激活检查")
    p.add_argument("--skip-mode-switch", action="store_true", help="不切换模式(仿真已在 walk)")
    p.add_argument("--no-stand-finish", action="store_true", help="结束时不回 stand")
    p.add_argument("--no-save-raw", action="store_true", help="不保存原始 CSV")
    p.add_argument("--timeout-wall", type=float, default=300.0, help="单项测试 wall 上限 (s)")
    p.add_argument("--selftest", action="store_true", help="合成数据自检, 不需要 ROS")
    return p


def main():
    args = build_parser().parse_args()

    if args.selftest:
        sys.exit(_selftest())
    if not args.test:
        print("请指定 --test {straight,turn,arc,stop} 或 --selftest")
        sys.exit(1)
    if args.levels is None:
        args.levels = DEFAULT_LEVELS[args.test]

    if not HAVE_ROS:
        print("错误: 未找到 rclpy, 无法执行仿真测试 (可用 --selftest)")
        sys.exit(3)

    rclpy.init()
    node = LocoPerfNode(args)
    print(f"═══ 运动控制基础性能测试: {args.test} ═══")
    print(f"  指令: {CMD_TOPIC}  真值: {GT_TOPIC}")
    print(f"  档位: {args.levels}  hold={args.hold}s window={args.window}s")
    try:
        node.sim_now(timeout_wall=30.0)
    except Exception as e:
        print(f"  [error] {e}")
        print("  提示: 仿真需使用 perf 配置 (pub_ground_truth_topic), 或直接使用 scripts/run_loco_perf.sh")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(3)
    rc, result = 3, None
    try:
        if args.test == "straight":
            rc, result = run_straight(node, args)
        elif args.test == "turn":
            rc, result = run_turn(node, args)
        elif args.test == "arc":
            rc, result = run_arc(node, args)
        else:
            rc, result = run_stop(node, args)
    except Exception as e:
        print(f"  [error] 测试中止: {e}")
        rc = 3
    finally:
        # 保存原始数据 (即使中途失败)
        base = (result or {}).get("_report_base")
        if base and not args.no_save_raw and node.gt:
            try:
                with open(base + ".gt.csv", "w", newline="") as f:
                    w = csv.writer(f)
                    w.writerow(["wall", "sim_time", "x", "y", "z", "roll", "pitch", "yaw",
                                "rtf", "collisions", "cum_dist"])
                    w.writerows(node.gt)
                with open(base + ".cmd.csv", "w", newline="") as f:
                    w = csv.writer(f)
                    w.writerow(["wall", "vx", "vy", "wz"])
                    w.writerows(node.cmd_log)
            except Exception as e:
                print(f"  [warn] 保存原始 CSV 失败: {e}")
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()
