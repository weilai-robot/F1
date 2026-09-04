#!/usr/bin/env python3
"""
analyze_nav_trials.py — 导航试验观测脚本

输入 reports/ 下的试验目录 (nav_test_runner 产物), 输出:
  1. 每试验 PNG 图 (matplotlib 缺失时自动跳过):
     trajectory.png  — GT 轨迹 vs 里程计轨迹 vs 目标点/起点
     cmd_vel.png     — vx/wz 时间线 (含 plan_time 标记)
     drift.png       — 定位漂移曲线 (含 p95/max 线)
     speed.png       — GT 差分速度剖面
  2. reports/observation_report.md — 全批次观测汇总:
     指标矩阵 + 异常检测 (失速/振荡/倒退/速度尖峰) + 门禁结论
  3. reports/observation.json      — 机器可读版

用法:
  python3 scripts/analyze_nav_trials.py --report-dir reports
"""

import argparse
import glob
import json
import math
import os
from datetime import datetime

# ── matplotlib 可选 ─────────────────────────────────────
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAS_MPL = True
except Exception:
    HAS_MPL = False

import numpy as np


def load_trial(trial_json: str) -> dict:
    d = {"result_path": trial_json, "trial_dir": os.path.dirname(trial_json)}
    with open(trial_json) as f:
        d["result"] = json.load(f)
    ts_path = os.path.join(d["trial_dir"], "timeseries.json")
    if os.path.exists(ts_path):
        with open(ts_path) as f:
            d["ts"] = json.load(f)
    else:
        d["ts"] = None
    return d


# ═══════════════════════════════════════════════════════════
#  异常检测 (从 timeseries 派生, 补充 result.json 指标)
# ═══════════════════════════════════════════════════════════
def detect_anomalies(trial: dict) -> dict:
    anomalies = {}
    ts = trial.get("ts")
    if not ts:
        return anomalies
    gt_rows = ts.get("ground_truth", {}).get("rows", [])
    cmd_rows = ts.get("cmd_vel", {}).get("rows", [])
    if len(gt_rows) >= 3:
        t = np.array([r[0] for r in gt_rows], dtype=float)
        x = np.array([r[1] for r in gt_rows], dtype=float)
        y = np.array([r[2] for r in gt_rows], dtype=float)
        dt = np.diff(t)
        dt[dt <= 0] = 1e-3
        v = np.hypot(np.diff(x), np.diff(y)) / dt

        # 1) 失速: 运动中速度 < 0.02 m/s 持续 ≥ 5s (排除起止各 1s)
        if len(v) > 10:
            stall_mask = v < 0.02
            # 简单游程统计
            stall_total = float(stall_mask.sum() * np.mean(dt))
            max_stall = 0.0
            run = 0.0
            for m_, d_ in zip(stall_mask, dt):
                run = run + d_ if m_ else 0.0
                max_stall = max(max_stall, run)
            anomalies["stall_max_s"] = round(max_stall, 2)
            anomalies["stall_total_s"] = round(stall_total, 2)

        # 2) 速度尖峰: 加速度超过 1.5 m/s² (MPPI ax_max=1.0)
        if len(v) >= 3:
            a = np.diff(v) / dt[1:]
            anomalies["accel_peak_m_s2"] = round(float(np.max(np.abs(a))), 2)

    if len(cmd_rows) >= 3:
        wt = np.array([r[0] for r in cmd_rows], dtype=float)
        wz = np.array([r[2] for r in cmd_rows], dtype=float)
        vx = np.array([r[1] for r in cmd_rows], dtype=float)
        dur = wt[-1] - wt[0]
        if dur > 1:
            # 3) wz 振荡: 符号翻转率 (|wz|>0.05 才计入)
            signs = np.sign(wz[np.abs(wz) > 0.05])
            flips = int(np.sum(np.diff(signs) != 0))
            anomalies["wz_sign_flips_per_sec"] = round(flips / dur, 3)
            # 4) cmd_vel 死区时间: 无指令占比
            idle = float(np.sum((np.abs(vx) < 0.005) & (np.abs(wz) < 0.005)) / len(vx))
            anomalies["cmd_idle_ratio"] = round(idle, 3)
    return anomalies


# ═══════════════════════════════════════════════════════════
#  绘图
# ═══════════════════════════════════════════════════════════
def plot_trial(trial: dict) -> list:
    if not HAS_MPL or not trial.get("ts"):
        return []
    ts = trial["ts"]
    trial_dir = trial["trial_dir"]
    made = []
    gt = ts.get("ground_truth", {}).get("rows", [])
    od = ts.get("odometry", {}).get("rows", [])
    cmd = ts.get("cmd_vel", {}).get("rows", [])
    paired = ts.get("odom_gt_paired", {}).get("rows", [])

    # 轨迹图
    if gt:
        gx = [r[1] for r in gt]; gy = [r[2] for r in gt]
        fig, ax = plt.subplots(figsize=(6, 6))
        ax.plot(gx, gy, "b-", lw=1.5, label="GT")
        if od:
            ax.plot([r[1] for r in od], [r[2] for r in od], "g--", lw=1.0, alpha=0.7, label="odom(SLAM)")
        res = trial["result"]; params = res.get("params", {}) or {}
        ax.plot(params.get("goal_x", 5.0), params.get("goal_y", 0.0), "r*", ms=18, label="goal")
        ax.plot(gx[0], gy[0], "ks", ms=8, label="start")
        ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.legend()
        m = res.get("metrics", {})
        ax.set_title(f"{res.get('scenario','?')}  {'PASS' if m.get('success') else 'FAIL'} "
                     f"err={m.get('position_error_m')}")
        p = os.path.join(trial_dir, "trajectory.png")
        fig.savefig(p, dpi=90, bbox_inches="tight"); plt.close(fig); made.append(p)

    # cmd_vel 时间线
    if cmd:
        t0 = cmd[0][0]
        fig, (a1, a2) = plt.subplots(2, 1, figsize=(8, 5), sharex=True)
        a1.plot([r[0] - t0 for r in cmd], [r[1] for r in cmd], "b-")
        a1.set_ylabel("vx (m/s)"); a1.grid(alpha=0.3)
        a2.plot([r[0] - t0 for r in cmd], [r[2] for r in cmd], "r-")
        a2.set_ylabel("wz (rad/s)"); a2.set_xlabel("t (s)"); a2.grid(alpha=0.3)
        meta = ts.get("meta", {})
        if meta.get("goal_sent_wall_t"):
            gs = meta["goal_sent_wall_t"] - t0
            for a in (a1, a2):
                a.axvline(gs, color="k", ls=":", lw=1)
        fig.suptitle("cmd_vel")
        p = os.path.join(trial_dir, "cmd_vel.png")
        fig.savefig(p, dpi=90, bbox_inches="tight"); plt.close(fig); made.append(p)

    # 漂移曲线
    if paired:
        fig, ax = plt.subplots(figsize=(8, 3))
        d = [r[5] for r in paired]
        ax.plot([r[0] for r in paired], d, "g-", lw=1)
        m = trial["result"].get("metrics", {})
        if m.get("drift_p95_m") is not None:
            ax.axhline(m["drift_p95_m"], color="orange", ls="--", lw=1, label=f"p95={m['drift_p95_m']}")
        if m.get("drift_max_m") is not None:
            ax.axhline(m["drift_max_m"], color="r", ls="--", lw=1, label=f"max={m['drift_max_m']}")
        ax.set_xlabel("sim t (s)"); ax.set_ylabel("drift (m)"); ax.grid(alpha=0.3); ax.legend()
        p = os.path.join(trial_dir, "drift.png")
        fig.savefig(p, dpi=90, bbox_inches="tight"); plt.close(fig); made.append(p)
    return made


# ═══════════════════════════════════════════════════════════
#  汇总
# ═══════════════════════════════════════════════════════════
METRIC_COLS = [
    ("position_error_m", "误差(m)"), ("yaw_err", "yaw误(rad)"),
    ("completion_time_s", "耗时(s)"), ("plan_time_s", "规划(s)"),
    ("path_efficiency", "路径效率"), ("drift_p95_m", "漂移p95(m)"),
    ("drift_max_m", "漂移max(m)"), ("vmax_m_s", "vmax"), ("vmean_m_s", "vmean"),
    ("linear_jerk_rms", "jerk线"), ("angular_jerk_rms", "jerk角"),
    ("rtf_mean", "RTF"), ("collisions", "碰撞"), ("fall", "摔"),
]


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def summarize(report_dir: str, make_plots: bool) -> dict:
    os.makedirs(report_dir, exist_ok=True)
    trials = sorted(glob.glob(os.path.join(report_dir, "*", "result.json")),
                    key=os.path.getmtime)
    obs = {"generated": datetime.now().isoformat(),
           "matplotlib": HAS_MPL, "trials": []}
    md = ["# 导航试验观测报告", ""]

    for tp in trials:
        try:
            trial = load_trial(tp)
        except Exception as e:
            continue
        res = trial["result"]
        m = res.get("metrics", {})
        params = res.get("params", {}) or {}
        anoms = detect_anomalies(trial)
        if make_plots:
            anoms["plots"] = [os.path.basename(p) for p in plot_trial(trial)]

        yaw_err = None
        if trial.get("ts"):
            rows = trial["ts"].get("ground_truth", {}).get("rows", [])
            if rows:
                tail = [r[6] for r in rows[-10:] if len(r) > 6]
                if tail:
                    yaw_med = sorted(tail)[len(tail) // 2]
                    yaw_err = round(abs(wrap_angle(yaw_med - float(params.get("goal_yaw", 0.0)))), 4)

        entry = {"scenario": res.get("scenario"), "trial_dir": trial["trial_dir"],
                 "metrics": m, "yaw_error_final_rad": yaw_err, "anomalies": anoms}
        obs["trials"].append(entry)

    # 指标矩阵
    md.append(f"共 {len(obs['trials'])} 次试验" + ("" if HAS_MPL or not make_plots else " (含图)"))
    md.append("")
    md.append("| 场景 | " + " | ".join(h for _, h in METRIC_COLS) +
              " | 失速max(s) | wz翻转/s |")
    md.append("|" + "---|" * (len(METRIC_COLS) + 3))
    for e in obs["trials"]:
        m = e["metrics"]
        vals = []
        for key, _ in METRIC_COLS:
            v = e.get("yaw_error_final_rad") if key == "yaw_err" else m.get(key)
            vals.append("N/A" if v is None else str(v))
        an = e["anomalies"]
        vals += [str(an.get("stall_max_s", "N/A")), str(an.get("wz_sign_flips_per_sec", "N/A"))]
        md.append(f"| {e['scenario']} | " + " | ".join(vals) + " |")

    # 异常高亮
    md.append("")
    md.append("## 异常与观测要点")
    for e in obs["trials"]:
        an = e["anomalies"]
        notes = []
        if an.get("stall_max_s", 0) and an["stall_max_s"] >= 5.0:
            notes.append(f"⚠️ 失速 {an['stall_max_s']}s")
        if an.get("wz_sign_flips_per_sec", 0) and an["wz_sign_flips_per_sec"] >= 1.0:
            notes.append(f"⚠️ wz 高频翻转 {an['wz_sign_flips_per_sec']}/s (振荡)")
        if an.get("accel_peak_m_s2", 0) and an["accel_peak_m_s2"] >= 1.5:
            notes.append(f"⚠️ GT 加速度尖峰 {an['accel_peak_m_s2']} m/s²")
        if e["metrics"].get("vmin_m_s") is not None and e["metrics"]["vmin_m_s"] < -0.05:
            notes.append(f"⚠️ 明显倒退 vmin={e['metrics']['vmin_m_s']}")
        md.append(f"- **{e['scenario']}**: " + ("; ".join(notes) if notes else "无明显异常"))

    out_json = os.path.join(report_dir, "observation.json")
    with open(out_json, "w") as f:
        json.dump(obs, f, indent=2, ensure_ascii=False)
    with open(os.path.join(report_dir, "observation_report.md"), "w") as f:
        f.write("\n".join(md) + "\n")
    print(f"[observation] {len(obs['trials'])} trials → {out_json}")
    return obs


def main():
    ap = argparse.ArgumentParser(description="导航试验观测脚本")
    ap.add_argument("--report-dir", default="reports")
    ap.add_argument("--no-plots", action="store_true")
    args = ap.parse_args()
    summarize(args.report_dir, make_plots=not args.no_plots)


if __name__ == "__main__":
    main()
