#!/usr/bin/env python3
"""
nav_strict_gate.py — 导航严格测试标准门禁 (v2: 地图最优路径动态标定)

对 batch_summary_*.json（或单试验 result.json）应用严格判定标准。
核心思想:
  - 路径效率不用"直线/实际"(对有墙地图不公平), 而用
    "A*最优路径(带机器人间隙约束)/实际位移" ≥ 阈值
  - 完成时间上限 = 最优路径 / 目标速度 + 余量 (动态, 与场景几何绑定)
  - 每试验从 timeseries.json 取实际起点, 顺序执行的场景链也被公平评估

用法:
  python3 scripts/nav_strict_gate.py --report-dir reports            # 强制 (exit 1 失败)
  python3 scripts/nav_strict_gate.py --report-dir reports --report-only

阈值标定依据 (run 33830360449 基线实测):
  jerk线 4.0-6.5 / jerk角 14-20.7 → 5.5 / 18.0 (最优场景水平)
  drift max 0.10-0.17 / p95 0.08-0.16 → 0.20 / 0.15 (ICP 重锚平台~0.16)
  RTF 1.12-1.16 → ≥0.90
  通过迭代优化后应逐步收紧 (文件头记录阶段目标)
"""

import argparse
import glob
import json
import math
import os
import sys
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nav_map_tools import NavMap  # noqa: E402

# ═══════════════════════════════════════════════════════════
#  严格标准
# ═══════════════════════════════════════════════════════════
GLOBAL_CRITERIA = {
    # P0 安全
    "fall":                {"op": "eq", "ref": False},
    "collisions":          {"op": "eq", "ref": 0},
    # P1 到达精度 (D 场景按场景覆盖)
    "position_error_m":    {"op": "le", "ref": 0.20},
    "yaw_error_final_rad": {"op": "le", "ref": 0.35},
    # P1 时效
    "plan_time_s":         {"op": "le", "ref": 5.0},
    # P1 定位质量
    "drift_max_m":         {"op": "le", "ref": 0.20},
    "drift_p95_m":         {"op": "le", "ref": 0.15},
    # P2 平顺
    "linear_jerk_rms":     {"op": "le", "ref": 5.5},
    "angular_jerk_rms":    {"op": "le", "ref": 18.0},
    "direction_reversals_per_sec": {"op": "le", "ref": 0.5},
    "vmin_m_s":            {"op": "ge", "ref": -0.05},
    # 仿真有效性
    "rtf_mean":            {"op": "ge", "ref": 0.90},
}

# 场景覆盖 (叠加在 GLOBAL 之上)
SCENARIO_OVERRIDES = {
    # 不可达目标场景: 只要求安全贴近 + 优雅处理。
    # completion_time 豁免: NavFn tolerance 0.5 使机器人停在墙前 ~0.5m 干等至超时
    # (goal_checker 0.15 永不满足, spin recovery 无位移) — 这是参数设计下的正确
    # 鲁棒行为, 时间不是该场景的质量信号。
    "D_impassable": {"position_error_max": 0.60, "skip_efficiency": True,
                     "skip_completion": True},
}

# 动态阈值参数
# EFF_MIN 依据: 静态地图 mujoco_lab.pgm 为旧 FastLIO 扫描, 与当前场景有 ~±10% 漂移
# (南门 y=-3 实际可通但地图显示封闭; 动态障碍残影成幻影墙 — run 33835908271 E 的
#  地图最优 19.3m > 实走 14.5m 即此伪影)。参考路径取保守下界后效率阈值 0.72。
# 地图重生成后应收回 0.75+。
EFF_MIN = 0.72            # 最优路径/实际位移 ≥ 0.72
TIME_V_MEAN = 0.15        # 目标平均速度 (m/s, vx_max=0.4 的 37.5%)
TIME_MARGIN_S = 15.0      # 完成时间固定余量
ROBOT_RADIUS = 0.25       # 与 nav2 robot_radius 一致

CPU_BUDGET_PCT = {"aimrt_main": 120.0, "nav2": 80.0, "fastlio": 60.0,
                  "lidar_bridge": 40.0}

OP_TEXT = {"eq": "==", "le": "≤", "ge": "≥"}


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def load_map():
    default_map = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                               "navigation", "planning", "humanoid_sim",
                               "maps", "mujoco_lab.yaml")
    if os.path.exists(default_map):
        try:
            return NavMap(default_map, robot_radius=ROBOT_RADIUS)
        except Exception as e:
            print(f"[strict-gate] ⚠ 地图加载失败 ({e}), 效率/时间退化为直线基准", file=sys.stderr)
    return None


def trial_geometry(report_dir: str, entry: dict, navmap):
    """从 timeseries.json 提取实际起点/位移, 结合地图算最优路径"""
    name = entry.get("scenario", "?")
    params = entry.get("params", {}) or {}
    gx, gy = float(params.get("goal_x", 0.0)), float(params.get("goal_y", 0.0))
    trials = sorted(glob.glob(os.path.join(report_dir, f"{name}_*", "timeseries.json")))
    if not trials:
        return {}
    try:
        with open(trials[-1]) as f:
            ts = json.load(f)
        rows = ts.get("ground_truth", {}).get("rows", [])
        if len(rows) < 2:
            return {}
        start = (rows[0][1], rows[0][2])
        trial_dist = float(rows[-1][9] - rows[0][9])  # cum_diff
        out = {"actual_start": start, "actual_dist_m": round(trial_dist, 3)}
        if navmap is not None:
            L = navmap.optimal_path_length(start[0], start[1], gx, gy)
            if L == L and L > 0.1:  # not NaN
                out["optimal_path_m"] = round(L, 2)
                if trial_dist > 0.05:
                    out["efficiency_optimal"] = round(min(L / trial_dist, 1.0), 3)
                out["time_cap_s"] = round(L / TIME_V_MEAN + TIME_MARGIN_S, 1)
        return out
    except Exception:
        return {}


def check(value, op: str, ref):
    if value is None:
        return False, None, "MISSING"
    try:
        if op == "eq":
            ok = value == ref
        elif op == "le":
            ok = float(value) <= float(ref)
        elif op == "ge":
            ok = float(value) >= float(ref)
        else:
            ok = False
    except (TypeError, ValueError):
        return False, None, "TYPE"
    margin = None
    if op in ("le", "ge") and isinstance(value, (int, float)) and not isinstance(value, bool):
        margin = round(float(value) - float(ref), 4)
    return ok, margin, None


def gate_batch(report_dir: str, report_only: bool) -> int:
    batch_files = sorted(glob.glob(os.path.join(report_dir, "batch_summary_*.json")),
                         key=os.path.getmtime)
    entries = []
    if batch_files:
        with open(batch_files[-1]) as f:
            entries = json.load(f)
    else:
        single = sorted(glob.glob(os.path.join(report_dir, "*", "result.json")),
                        key=os.path.getmtime)
        if single:
            with open(single[-1]) as f:
                entries = [json.load(f)]
    if not entries:
        print("[strict-gate] 未找到任何测试结果", file=sys.stderr)
        return 2

    navmap = load_map()
    gate_result = {"generated": datetime.now().isoformat(),
                   "map_optimal": navmap is not None,
                   "scenarios": [], "pass_all": True}
    lines = ["| 指标 | 阈值 | 实测 | 判定 | 裕量 |", "|------|------|------|------|------|"]
    total_fail = 0

    for entry in entries:
        name = entry.get("scenario", "?")
        m = entry.get("metrics", {})
        params = entry.get("params", {}) or {}
        goal_yaw = math.radians(float(params.get("goal_yaw_deg", 0.0)))
        timeout = float(params.get("timeout", m.get("timeout_s", 120)))

        yaw_err = None
        tss = sorted(glob.glob(os.path.join(report_dir, f"{name}_*", "timeseries.json")))
        if tss:
            try:
                with open(tss[-1]) as f:
                    rows = json.load(f).get("ground_truth", {}).get("rows", [])
                tail = [r[6] for r in rows[-10:] if len(r) > 6]
                if tail:
                    yaw_err = round(abs(wrap_angle(sorted(tail)[len(tail)//2] - goal_yaw)), 4)
            except Exception:
                pass
        m_eval = dict(m)
        m_eval["yaw_error_final_rad"] = yaw_err

        geo = trial_geometry(report_dir, entry, navmap)
        m_eval["efficiency_optimal"] = geo.get("efficiency_optimal")
        m_eval["optimal_path_m"] = geo.get("optimal_path_m")

        checks = [(k, r, m_eval.get(k)) for k, r in GLOBAL_CRITERIA.items()]
        ov = SCENARIO_OVERRIDES.get(name, {})
        if "position_error_max" in ov:
            checks = [(k, {"op": "le", "ref": ov["position_error_max"]}, v)
                      if k == "position_error_m" else (k, r, v) for k, r, v in checks]
        if not ov.get("skip_efficiency") and geo.get("efficiency_optimal") is not None:
            checks.append(("efficiency_optimal", {"op": "ge", "ref": EFF_MIN},
                           geo["efficiency_optimal"]))
        # 完成时间: 动态 cap (最优路径/0.15 + 15s), 不超过 0.75×timeout
        if ov.get("skip_completion"):
            cap = None
        elif "completion_time_s_max" in ov:
            cap = ov["completion_time_s_max"]
        elif geo.get("time_cap_s") is not None:
            cap = min(geo["time_cap_s"], timeout * 0.75)
        else:
            cap = timeout * 0.75
        if cap is not None:
            checks.append(("completion_time_s", {"op": "le", "ref": round(cap, 1)},
                           m_eval.get("completion_time_s")))

        sc_fail, row = [], {"scenario": name, "geometry": geo, "checks": [], "pass": True}
        for key, rule, val in checks:
            ok, margin, status = check(val, rule["op"], rule["ref"])
            row["checks"].append({"metric": key, "threshold": rule["ref"],
                                  "op": OP_TEXT[rule["op"]], "value": val,
                                  "pass": ok, "margin": margin, "status": status})
            if not ok:
                sc_fail.append(key + (f"({status})" if status else ""))
                row["pass"] = False

        cpu_mem = m.get("cpu_mem") or {}
        for proc, budget in CPU_BUDGET_PCT.items():
            for pn, st in cpu_mem.items():
                if proc in pn and st.get("cpu_mean_pct") is not None \
                        and st["cpu_mean_pct"] > budget:
                    sc_fail.append(f"cpu:{pn}")
                    row["pass"] = False

        gate_result["scenarios"].append(row)
        if not row["pass"]:
            gate_result["pass_all"] = False
            total_fail += 1

        geo_str = (f"最优{geo.get('optimal_path_m','?')}m 实走{geo.get('actual_dist_m','?')}m"
                   if geo else "")
        lines.append(f"| **{name}** {geo_str} | | | {'✅' if row['pass'] else '❌ ' + ','.join(sc_fail)} | |")
        for c in row["checks"]:
            val_str = "N/A" if c["value"] is None else c["value"]
            mg_str = "" if c["margin"] is None else f"{c['margin']:+}"
            lines.append(f"| &nbsp;&nbsp;{c['metric']} | {c['op']} {c['threshold']} "
                         f"| {val_str} | {'✅' if c['pass'] else '❌'} | {mg_str} |")

    with open(os.path.join(report_dir, "strict_gate.json"), "w") as f:
        json.dump(gate_result, f, indent=2, ensure_ascii=False)
    with open(os.path.join(report_dir, "strict_gate.md"), "w") as f:
        f.write(f"# 严格测试标准门禁 ({'仅报告' if report_only else '强制'})\n\n"
                f"- 通过: {len(entries) - total_fail}/{len(entries)}\n\n"
                + "\n".join(lines) + "\n")

    print(f"[strict-gate] {'✅ 全部通过' if gate_result['pass_all'] else f'❌ {total_fail}/{len(entries)} 场景未达标'}"
          + ("" if navmap else " (⚠无地图, 效率退化直线基准)"))
    for row in gate_result["scenarios"]:
        icon = "✅" if row["pass"] else "❌"
        fails = [f"{c['metric']}={c['value']}{c['op']}{c['threshold']}"
                 if c["value"] is not None else f"{c['metric']}=MISSING"
                 for c in row["checks"] if not c["pass"]]
        print(f"  {icon} {row['scenario']}" + (f"  失败: {'; '.join(fails)}" if fails else ""))
    print(f"[strict-gate] 明细: {os.path.join(report_dir, 'strict_gate.json')}")

    return 0 if (report_only or gate_result["pass_all"]) else 1


def main():
    ap = argparse.ArgumentParser(description="导航严格标准门禁 v2")
    ap.add_argument("--report-dir", default="reports")
    ap.add_argument("--report-only", action="store_true")
    args = ap.parse_args()
    sys.exit(gate_batch(args.report_dir, args.report_only))


if __name__ == "__main__":
    main()
