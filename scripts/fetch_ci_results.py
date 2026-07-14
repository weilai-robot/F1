#!/usr/bin/env python3
"""
fetch_ci_results.py — 获取 CI 导航评测结果 + 生成 Agent 诊断报告

用法:
  # Agent push 后调用 (阻塞等待 CI 完成, 然后下载分析)
  python3 scripts/fetch_ci_results.py

  # 指定 run_id (不等待, 直接下载)
  python3 scripts/fetch_ci_results.py --run-id 12345678

  # 只看最近一次结果 (不等待)
  python3 scripts/fetch_ci_results.py --latest

输出:
  ci_fetched/          — 下载的原始 artifact 文件
  ci_fetched/report.md — 诊断报告 (Agent 直接读取)
  stdout               — 诊断摘要 (Agent 直接消费)

前提:
  gh CLI 已安装并已认证 (gh auth login)
"""
import argparse
import json
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path


# ── gh CLI 封装 ────────────────────────────────────────────
def gh(*args, check=True, capture=True):
    """调用 gh CLI"""
    cmd = ["gh"] + list(args)
    result = subprocess.run(cmd, capture_output=capture, text=True)
    if check and result.returncode != 0:
        print(f"[ERROR] gh {' '.join(args)} 失败 (exit={result.returncode})")
        if result.stderr:
            print(f"  stderr: {result.stderr[:500]}")
        sys.exit(1)
    return result.stdout.strip()


def find_latest_run(workflow="nav_eval.yml", branch=None):
    """找最新的 nav_eval workflow run"""
    args = ["run", "list", f"--workflow={workflow}", "--limit=1", "--json=databaseId,status,conclusion,headBranch,createdAt,displayTitle"]
    if branch:
        args.append(f"--branch={branch}")
    out = gh(*args)
    runs = json.loads(out)
    if not runs:
        return None
    return runs[0]


def wait_for_run(run_id, timeout=2400, poll_interval=30):
    """等待 run 完成 (默认 40 分钟超时)"""
    print(f"[fetch] 等待 run {run_id} 完成 (超时 {timeout//60} 分钟)...")
    start = time.monotonic()
    while True:
        out = gh("run", "view", str(run_id), "--json=status,conclusion")
        info = json.loads(out)
        status = info.get("status", "")
        conclusion = info.get("conclusion", "")

        if status == "completed":
            print(f"[fetch] run {run_id} 完成: conclusion={conclusion}")
            return conclusion

        elapsed = int(time.monotonic() - start)
        print(f"  [{elapsed}s] status={status}...", flush=True)

        if elapsed >= timeout:
            print(f"[ERROR] 等待超时 ({timeout}s)")
            sys.exit(1)

        time.sleep(poll_interval)


def download_artifacts(run_id, dest_dir):
    """下载 run 的所有 artifact"""
    # 列出 artifact
    out = gh("run", "view", str(run_id), "--json=artifacts")
    info = json.loads(out)
    artifacts = info.get("artifacts", [])

    if not artifacts:
        print(f"[ERROR] run {run_id} 没有 artifact")
        sys.exit(1)

    print(f"[fetch] 发现 {len(artifacts)} 个 artifact:")
    for a in artifacts:
        print(f"  - {a['name']} ({a.get('sizeInBytes', '?')} bytes)")

    # 下载
    os.makedirs(dest_dir, exist_ok=True)
    for a in artifacts:
        name = a["name"]
        # gh run download 到临时目录再解压
        tmp = os.path.join(dest_dir, f"_tmp_{name}")
        os.makedirs(tmp, exist_ok=True)
        gh("run", "download", str(run_id), f"--name={name}", f"--dir={tmp}",
           check=False)  # 有些 artifact 可能不存在
        # 合并到 dest_dir
        merge_dir(tmp, dest_dir)
        # 清理临时目录
        import shutil
        shutil.rmtree(tmp, ignore_errors=True)

    print(f"[fetch] artifact 已下载到 {dest_dir}")


def merge_dir(src, dst):
    """合并 src 内容到 dst"""
    for item in os.listdir(src):
        s = os.path.join(src, item)
        d = os.path.join(dst, item)
        if os.path.isdir(s):
            import shutil
            shutil.copytree(s, d, dirs_exist_ok=True)
        else:
            import shutil
            shutil.copy2(s, d)


# ── 诊断分析 ────────────────────────────────────────────────
def analyze_results(results_dir):
    """分析所有结果文件, 生成诊断报告"""
    report_lines = []
    report_lines.append("# CI 导航评测诊断报告")
    report_lines.append("")

    # ── 收集所有 JSON ──
    test_results = []
    batch_summary = None
    ci_diag = None

    for f in sorted(Path(results_dir).glob("*.json")):
        try:
            data = json.loads(f.read_text())
        except Exception:
            continue

        if f.name.startswith("batch_summary"):
            batch_summary = data
        elif f.name == "ci_diagnostic.json":
            ci_diag = data
        elif "metrics" in data:
            test_results.append(data)

    if not test_results and batch_summary:
        test_results = batch_summary

    if not test_results:
        report_lines.append("⚠ 未找到测试结果文件")
        return "\n".join(report_lines)

    # ── 1. 总览 ──
    report_lines.append("## 总览")
    report_lines.append("")
    total = len(test_results)
    passed = sum(1 for r in test_results if r.get("metrics", {}).get("success"))
    report_lines.append(f"- 测试场景: {total}")
    report_lines.append(f"- 通过: {passed}/{total}")
    report_lines.append("")

    # ── 2. 逐场景分析 ──
    report_lines.append("## 场景结果")
    report_lines.append("")
    report_lines.append("| 场景 | 结果 | 摔倒 | 碰撞 | 位置误差(m) | SLAM漂移mean(m) | 线Jerk | RTF |")
    report_lines.append("|------|------|------|------|-----------|---------------|--------|-----|")

    for r in test_results:
        m = r.get("metrics", {})
        name = r.get("scenario", "?")
        succ = "✅" if m.get("success") else "❌"
        fall = "是" if m.get("fall") else "否"
        col = m.get("collisions", "?")
        err = m.get("position_error_m", "?")
        drift = m.get("drift_mean_m", "?")
        jerk = m.get("linear_jerk_rms", "?")
        rtf = m.get("rtf_mean", "?")
        report_lines.append(f"| {name} | {succ} | {fall} | {col} | {err} | {drift} | {jerk} | {rtf} |")

    report_lines.append("")

    # ── 3. 失败原因分析 ──
    failed = [r for r in test_results if not r.get("metrics", {}).get("success")]
    if failed:
        report_lines.append("## 失败原因分析")
        report_lines.append("")

        for r in failed:
            m = r.get("metrics", {})
            diag = r.get("diagnostics", {})
            name = r.get("scenario", "?")
            report_lines.append(f"### {name}")
            report_lines.append("")

            # 摔倒分析
            if m.get("fall"):
                fa = diag.get("fall_analysis", {})
                report_lines.append(f"- **摔倒**: t={fa.get('fall_time_s', '?')}s, "
                                    f"pitch={fa.get('fall_pitch_deg', '?')}°, "
                                    f"roll={fa.get('fall_roll_deg', '?')}°, "
                                    f"z={fa.get('fall_z_m', '?')}m")
                vbf = fa.get("velocity_before_fall", [])
                if vbf:
                    report_lines.append(f"  - 摔倒前速度: {vbf}")

                # 判断摔倒类型
                pitch = fa.get("fall_pitch_deg", 0)
                roll = fa.get("fall_roll_deg", 0)
                if abs(pitch) > abs(roll):
                    report_lines.append("  - **类型: 前后倒 (pitch)** — 可能原因: 步态不稳定 / 速度突变 / 减速不当")
                else:
                    report_lines.append("  - **类型: 侧倒 (roll)** — 可能原因: 横向不稳定 / 转弯过急 / 重心偏移")

            # 碰撞
            if m.get("collisions", 0) > 0:
                report_lines.append(f"- **碰撞**: {m['collisions']} 次")

            # SLAM 漂移
            drift_mean = m.get("drift_mean_m")
            if drift_mean is not None and drift_mean > 0.5:
                drift_curve = diag.get("drift_curve", [])
                report_lines.append(f"- **SLAM 漂移严重**: mean={drift_mean}m max={m.get('drift_max_m', '?')}m")
                if drift_curve:
                    first_d = drift_curve[0][1] if drift_curve else 0
                    last_d = drift_curve[-1][1] if drift_curve else 0
                    mid_d = drift_curve[len(drift_curve)//2][1] if drift_curve else 0
                    if last_d > mid_d * 2 > first_d * 5:
                        report_lines.append("  - **漂移趋势: 指数发散** — FastLIO2 里程计积分发散, 可能原因: LiDAR 数据质量差 / IMU 偏置 / 时间同步错误")
                    elif last_d > first_d * 10 and abs(last_d - mid_d) < last_d * 0.3:
                        report_lines.append("  - **漂移趋势: 线性增长** — 可能原因: 标定误差 / 常值偏置积累")
                    elif any(abs(drift_curve[i][1] - drift_curve[i-1][1]) > 50 for i in range(1, len(drift_curve))):
                        report_lines.append("  - **漂移趋势: 突变跳变** — ICP 定位回环或跳变, 可能原因: 初始定位错误 / 点云匹配失败")

            # cmd_vel 异常
            jerk = m.get("linear_jerk_rms")
            if jerk is not None and jerk > 5.0:
                report_lines.append(f"- **速度控制不稳定**: 线Jerk={jerk} m/s³ (正常<2.0)")
                report_lines.append("  - 可能原因: Nav2 控制器参数 / 机器人跟踪响应振荡 / costmap 抖动")

            # Nav2 结果
            status = m.get("result_status", "?")
            if "FAILED" in str(status) or "TIMEOUT" in str(status):
                report_lines.append(f"- **Nav2 状态**: {status}")
                if "TIMEOUT" in str(status):
                    report_lines.append("  - 导航超时: 机器人未能到达目标, 可能原因: 卡在障碍物前 / 路径规划反复 / 速度太慢")

            report_lines.append("")

    # ── 4. 组件日志诊断 ──
    if ci_diag:
        comp_logs = ci_diag.get("component_logs", {})
        if comp_logs:
            report_lines.append("## 组件日志诊断")
            report_lines.append("")
            report_lines.append("| 组件 | Errors | Warnings |")
            report_lines.append("|------|--------|----------|")

            problem_components = []
            for comp, info in comp_logs.items():
                errs = info.get("error_count", 0)
                warns = info.get("warn_count", 0)
                icon = "❌" if errs > 0 else "✅"
                report_lines.append(f"| {icon} {comp} | {errs} | {warns} |")
                if errs > 0:
                    problem_components.append((comp, info))

            report_lines.append("")

            # 列出有 error 的组件的具体错误
            for comp, info in problem_components:
                errors = info.get("errors", [])
                if errors:
                    report_lines.append(f"### {comp} 错误详情 ({len(errors)} 条, 显示前 5 条)")
                    report_lines.append("```")
                    for e in errors[:5]:
                        report_lines.append(e)
                    report_lines.append("```")
                    report_lines.append("")

    # ── 5. 关键事件时间线 ──
    if test_results:
        diag = test_results[0].get("diagnostics", {})
        events = diag.get("events", [])
        if events:
            report_lines.append("## 事件时间线 (首个场景)")
            report_lines.append("")
            for ev in events:
                t = ev.get("t", "?")
                etype = ev.get("type", "?")
                desc = ev.get("desc", "")
                report_lines.append(f"- t={t}s **{etype}**: {desc}")
            report_lines.append("")

    return "\n".join(report_lines)


# ── 主流程 ──────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="获取 CI 导航评测结果")
    parser.add_argument("--run-id", type=int, help="指定 run ID (跳过等待)")
    parser.add_argument("--latest", action="store_true", help="下载最近一次结果 (不等新 run)")
    parser.add_argument("--branch", type=str, help="指定分支")
    parser.add_argument("--timeout", type=int, default=2400, help="等待超时 (秒)")
    parser.add_argument("--dest", type=str, default="ci_fetched", help="下载目录")
    parser.add_argument("--no-wait", action="store_true", help="不等 run 完成, 直接下载")
    args = parser.parse_args()

    # 检查 gh CLI
    if not shutil_which("gh"):
        print("[ERROR] gh CLI 未安装")
        print("  安装: https://cli.github.com/")
        sys.exit(1)

    # 检查认证
    auth = gh("auth", "status", check=False)
    if "Logged in" not in auth and "account" not in auth.lower():
        print("[ERROR] gh CLI 未认证")
        print("  运行: gh auth login")
        sys.exit(1)

    # 1. 找 run
    if args.run_id:
        run_id = args.run_id
        print(f"[fetch] 使用指定 run_id: {run_id}")
    else:
        run = find_latest_run(branch=args.branch)
        if not run:
            print("[ERROR] 未找到 nav_eval workflow run")
            print("  请先触发 CI: push 到 nav/dev-* 分支 或手动 dispatch")
            sys.exit(1)
        run_id = run["databaseId"]
        status = run["status"]
        print(f"[fetch] 最新 run: {run_id} (status={status}, branch={run.get('headBranch', '?')})")
        print(f"  title: {run.get('displayTitle', '?')}")

    # 2. 等待完成
    if not args.no_wait:
        conclusion = wait_for_run(run_id, timeout=args.timeout)
    else:
        out = gh("run", "view", str(run_id), "--json=status,conclusion")
        conclusion = json.loads(out).get("conclusion", "unknown")

    # 3. 下载 artifact
    dest = os.path.abspath(args.dest)
    download_artifacts(run_id, dest)

    # 4. 分析
    print("\n[fetch] 分析结果...\n")
    report = analyze_results(dest)

    # 保存报告
    report_path = os.path.join(dest, "report.md")
    with open(report_path, "w") as f:
        f.write(report)

    # 输出到 stdout
    print(report)
    print(f"\n[fetch] 报告已保存: {report_path}")
    print(f"[fetch] 原始数据: {dest}/")


def shutil_which(cmd):
    """shutil.which 的简单实现"""
    for path in os.environ.get("PATH", "").split(os.pathsep):
        full = os.path.join(path, cmd)
        if os.path.isfile(full) and os.access(full, os.os.X_OK):
            return full
    return None


if __name__ == "__main__":
    main()
