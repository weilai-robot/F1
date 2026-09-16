#!/usr/bin/env python3
"""nav_debug_plot.py 离线自检 — 纯函数 + 合成数据全链路 (不需要 ROS)。

运行:
  python3 scripts/tests/test_nav_debug_plot.py

覆盖:
  1. 纯函数: 四元数转 yaw / 角度插值 / 位姿序列插值 / 点集变换 / 区间检测 / 重叠 / 平滑
  2. 合成数据全链路: 卡停&零指令检测、map 系变换、CSV/PNG/summary 生成

输出写入临时目录, 不污染 reports/。
"""
import importlib.util
import math
import os
import shutil
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MOD_PATH = os.path.abspath(os.path.join(HERE, "..", "nav_debug_plot.py"))


def load_module():
    spec = importlib.util.spec_from_file_location("nav_debug_plot", MOD_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main():
    if not os.path.isfile(MOD_PATH):
        print(f"[ERROR] 找不到 {MOD_PATH}", file=sys.stderr)
        return 1

    mod = load_module()
    fails = []

    def check(name, cond):
        print(("  [OK] " if cond else "  [FAIL] ") + name)
        if not cond:
            fails.append(name)

    print("== 1. yaw_from_quaternion ==")
    check("identity -> 0", abs(mod.yaw_from_quaternion(0, 0, 0, 1)) < 1e-12)
    qy = math.sin(0.25 * math.pi)
    check(
        "90deg -> pi/2",
        abs(
            mod.yaw_from_quaternion(0, 0, qy, math.cos(0.25 * math.pi)) - math.pi / 2
        )
        < 1e-9,
    )

    print("== 2. lerp_angle 最短路径 ==")
    check("跨 ±pi", abs(mod.lerp_angle(3.0, -3.0, 0.5)) > 3.0)

    print("== 3. PoseSeries 插值/外推 ==")
    ps = mod.PoseSeries([0.0, 1.0, 2.0], [0.0, 1.0, 2.0], [0.0, 0.0, 0.0], [0.0, 0.1, 0.2])
    x, y, yaw, cl = ps.at(0.5)
    check("中点插值", abs(x - 0.5) < 1e-9 and abs(yaw - 0.05) < 1e-9 and not cl)
    x, y, yaw, cl = ps.at(3.0)
    check("越界外推标记", cl and abs(x - 2.0) < 1e-9)
    check("边界点不误报越界", ps.at(2.0)[3] is False)
    check("计数", ps.n_clamped == 1 and ps.n_queries == 3)

    print("== 4. transform_points ==")
    pts = np.array([[1.0, 0.0]])
    out = mod.transform_points(pts, 10.0, 20.0, math.pi / 2)
    check("平移+90°旋转", abs(out[0, 0] - 10.0) < 1e-9 and abs(out[0, 1] - 21.0) < 1e-9)

    print("== 5. find_segments ==")
    ts = np.arange(0, 10, 0.5)
    mask = np.zeros_like(ts, dtype=bool)
    mask[4:12] = True  # t=2.0~5.5 持续 3.5s
    segs = mod.find_segments(mask, ts, min_dur=1.0)
    check(
        "单区间",
        len(segs) == 1 and abs(segs[0][0] - 2.0) < 1e-9 and abs(segs[0][1] - 5.5) < 1e-9,
    )
    mask2 = np.zeros_like(ts, dtype=bool)
    mask2[2] = True
    check("单样本被过滤", mod.find_segments(mask2, ts, min_dur=1.0) == [])
    check(
        "过短被过滤",
        mod.find_segments(
            np.r_[np.zeros(2, bool), np.ones(2, bool), np.zeros(6, bool)], ts, min_dur=2.0
        )
        == [],
    )

    print("== 6. overlap_segments ==")
    ov = mod.overlap_segments([(1.0, 5.0, 0, 0)], [(3.0, 8.0, 0, 0)])
    check("交集", len(ov) == 1 and ov[0] == (3.0, 5.0))
    check("无交集", mod.overlap_segments([(1.0, 2.0, 0, 0)], [(3.0, 4.0, 0, 0)]) == [])

    print("== 7. moving_average ==")
    x = np.array([0.0, 0.0, 10.0, 0.0, 0.0])
    s = mod.moving_average(x, 3)
    check("平滑峰值降低", s[2] < 10.0 and s[1] > 0.0)

    # ============ 合成数据 -> 全链路 ============
    print("== 8. 合成数据全链路 ==")
    t0 = 1_700_000_000.0
    HZ = 10.0
    N = 100
    ts = t0 + np.arange(N) / HZ

    tf = [(t, 1.0, 2.0, math.radians(5.0)) for t in ts]  # map->odom 恒定

    # odom: 前40帧走到 x=2.3, 40 帧后卡停。
    # twist 恒 0 — 模拟 FastLIO 不填充 twist 字段的真实情况
    # (卡停检测现在必须依赖位姿差分)。
    odom = []
    px = 0.0
    for i, t in enumerate(ts):
        if i > 0 and i <= 40:
            px = 2.3 * i / 40.0
        odom.append((t, px, 0.0, 0.0, 0.0, 0.0))

    cmd_lim = [(t, (0.30 if i < 40 else 0.02), 0.0) for i, t in enumerate(ts)]
    cmd_raw = [(t, (0.32 if i < 40 else 0.05), 0.0) for i, t in enumerate(ts)]

    plan_pts = np.array([[1.0 + 0.25 * k, 2.0] for k in range(21)])
    plans = [(ts[0], "map", plan_pts)]

    # MPPI 跟踪参考段 (odom 系, 每 10 帧一条)
    ref_plans = []
    for i in range(0, N, 10):
        seg = np.array([[k * 0.1, 0.0] for k in range(10)])
        ref_plans.append((ts[i], "odom", seg))

    # MPPI 最优轨迹 (odom 系), 卡停后变短
    trajs = []
    for i in range(0, N, 2):
        L = 32
        end = 3.2 if i < 40 else 0.2
        traj_pts = np.array([[k * end / L, 0.0] for k in range(L)])
        trajs.append((ts[i], "odom", traj_pts))

    occ = np.zeros(80 * 80, dtype=np.int8)
    occ[(40 * 80 + 60) : (40 * 80 + 66)] = 100
    costmaps = [(ts[0], 0.0, 0.0, 0.05, 80, 80, occ)]

    # FastLIO 原始里程计 (camera_init 系, 与 /odom 趋势一致即可)
    fl_odom = [(o[0], o[1], o[2], o[3]) for o in odom]

    # ICP: 前 60 帧 fitness=0.95, 之后掉到 0.7 (冻结); 第 65 帧矩阵跳变 (模拟冻结后重锁)
    fitness = [(t, 0.95 if i < 60 else 0.70) for i, t in enumerate(ts)]
    odom2map = []
    for i, t in enumerate(ts):
        x = 1.0 if i < 65 else 1.35          # 跳 0.35m
        yaw = math.radians(5.0) if i < 65 else math.radians(9.0)  # 跳 4°
        odom2map.append((t, x, 2.0, yaw))

    # 模式事件: t0+0s stand, t0+5s walk (重发模拟), t0+25s stand
    mode_events = [
        (ts[0], "stand"),
        (ts[0] + 0.4, "stand"),   # 重复发布, 应被合并
        (ts[0] + 0.8, "stand"),   # 重复发布, 应被合并
        (ts[50], "walk"),
        (ts[90], "stand"),
    ]

    # tf_map_odom 用 odom2map 的数据 (含跳变) — 让 TF 阶跃检测有对象
    tf_jumpy = odom2map

    data = {
        "plans": plans,
        "trajs": trajs,
        "ref_plans": ref_plans,
        "cmd_raw": cmd_raw,
        "cmd_lim": cmd_lim,
        "odom": odom,
        "fl_odom": fl_odom,
        "odom2map": odom2map,
        "fitness": fitness,
        "mode_events": mode_events,
        "tf_map_odom": tf_jumpy,
        "costmaps": costmaps,
        "topic_counts": {"synthetic": 1},
        "warnings": [],
    }

    res = mod.compute_analysis(
        data, stuck_eps=0.05, stuck_min_dur=1.0, zero_eps=0.05, zero_min_dur=1.0
    )
    check("检测到零指令区间", len(res["zero_cmd_segs"]) >= 1)
    check("检测到卡停区间", len(res["robot"]["stuck_segs"]) >= 1)
    check("检测到重叠区间", len(res["zero_and_stuck"]) >= 1)
    check("检测到 vx≈0∩卡停 区间", len(res["vx0_and_stuck"]) >= 1)

    zc = res["zero_cmd_segs"][0]
    check("零指令区间起点 ≈4s", abs((zc[0] - t0) - 3.9) < 0.35)

    # 位姿差分速度 (twist=0 情况下的关键路径)
    vmax_pose = float(np.max(res["robot"]["vx_meas"]))
    check("位姿差分速度≈0.58 (行走段)", abs(vmax_pose - 0.575) < 0.12)
    check("twist≈0 时给出告警", any("twist" in w and "位姿差分" in w for w in res["warnings"]))

    rm = res["robot"]
    i40 = 40
    exp = mod.transform_points(
        np.array([[rm["x_odom"][i40], rm["y_odom"][i40]]]), 1.0, 2.0, math.radians(5.0)
    )
    check(
        "map 变换正确",
        abs(rm["x_map"][i40] - exp[0, 0]) < 1e-6 and abs(rm["y_map"][i40] - exp[0, 1]) < 1e-6,
    )

    check("MPPI 参考段已转 map 系", res.get("ref_map") is not None)
    if res.get("ref_map") is not None:
        ref_t, ref_pts = res["ref_map"]
        ps = res["vx0_and_stuck"][0]
        check("参考段取自最可疑时段", ps[0] <= ref_t <= ps[1] + 0.2)

    # ---- 场景 B: 卡停但原地转向 (wz≠0) — 严格"指令全零"不应命中, vx≈0 应命中 ----
    cmd_lim_b = [
        (t, (0.30 if i < 40 else 0.02), (0.0 if i < 40 else 0.30))
        for i, t in enumerate(ts)
    ]
    data_b = dict(data)
    data_b["cmd_lim"] = cmd_lim_b
    res_b = mod.compute_analysis(
        data_b, stuck_eps=0.05, stuck_min_dur=1.0, zero_eps=0.05, zero_min_dur=1.0
    )
    check("场景B: 无严格零指令区间 (wz≠0)", len(res_b["zero_cmd_segs"]) == 0)
    check("场景B: 仍检测到 vx≈0∩卡停", len(res_b["vx0_and_stuck"]) >= 1)

    # ---- 新判据: TF 阶跃 / fitness 冻结 / 模式去重 ----
    check("TF 跳变被检测到 (0.35m/4°)", len(res["tf_jumps"]) >= 1)
    if res["tf_jumps"]:
        tj = res["tf_jumps"][0]
        check("TF 跳变量级正确", tj[1] > 0.3 and tj[2] > 3.5 and abs((tj[0] - t0) - 6.5) < 0.2)
    check("fitness 统计存在", res["fitness_stats"] is not None)
    if res["fitness_stats"]:
        check("fitness 低占比 ≈40%", abs(res["fitness_stats"]["low_ratio"] - 0.40) < 0.05)
    check("冻结区间被检测到", len(res["frozen_segs"]) >= 1)
    check("模式事件去重 (5→3)", len(res["mode_events"]) == 3)
    check(
        "模式序列正确",
        [m for _, m in res["mode_events"]] == ["stand", "walk", "stand"],
    )

    # ---- 场景 C: RotationShim 原地旋转 (wz 恒值 + MPPI 轨迹静默) ----
    trajs_quiet = [(t, f, p) for (t, f, p), i in zip(trajs, range(0, N, 2)) if i < 40 or i > 72]
    cmd_lim_c = [
        (t, (0.30 if i < 40 else 0.0), (0.0 if i < 40 else 0.35))
        for i, t in enumerate(ts)
    ]
    data_c = dict(data)
    data_c["cmd_lim"] = cmd_lim_c
    data_c["trajs"] = trajs_quiet
    res_c = mod.compute_analysis(
        data_c, stuck_eps=0.05, stuck_min_dur=1.0, zero_eps=0.05, zero_min_dur=1.0
    )
    check("场景C: Shim 原地旋转区间被检测到", len(res_c["shim_segs"]) >= 1)
    if res_c["shim_segs"]:
        seg = res_c["shim_segs"][0]
        check("场景C: Shim 静默起点 ≈3.8s", abs((seg[0] - t0) - 3.8) < 0.3)
    # 对照: 原始数据 (轨迹全程覆盖) 不应误报 Shim
    check("主场景: 无 Shim 误报 (轨迹全程覆盖)", len(res["shim_segs"]) == 0)

    out_dir = tempfile.mkdtemp(prefix="nav_debug_selftest_")
    try:
        data_dir = os.path.join(out_dir, "data")
        os.makedirs(data_dir, exist_ok=True)
        csvs = mod.export_csvs(data, res, data_dir)
        check("CSV 生成", len(csvs) >= 3 and all(os.path.isfile(p) for p in csvs))

        pngs = mod.make_plots(data, res, out_dir, max_traj_slices=10)
        check("PNG 生成 (matplotlib 可用时)", all(os.path.isfile(p) for p in pngs))

        summary_path = os.path.join(out_dir, "summary.md")
        mod.write_summary(data, res, "synthetic_run", summary_path)
        txt = open(summary_path).read()
        check("摘要含最可疑段", "最可疑" in txt)
        check("摘要含数据质量节", "## 数据质量" in txt)
    finally:
        shutil.rmtree(out_dir, ignore_errors=True)

    print()
    if fails:
        print(f"[FAILED] {len(fails)} 项: {fails}")
        return 1
    print("[PASS] 全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
