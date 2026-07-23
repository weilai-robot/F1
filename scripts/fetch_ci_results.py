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
import time
from pathlib import Path

from nav_eval_manifest import ManifestError, verify_manifest


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


def find_latest_run(workflow="nav_eval.yml", branch=None, expected_sha=None):
    """找最新的 nav_eval workflow run，可限定精确 F1 commit。"""
    args = [
        "run", "list", f"--workflow={workflow}", "--limit=30",
        "--json=databaseId,status,conclusion,headBranch,headSha,createdAt,displayTitle",
    ]
    if branch:
        args.append(f"--branch={branch}")
    out = gh(*args)
    runs = json.loads(out)
    if expected_sha:
        runs = [run for run in runs if run.get("headSha") == expected_sha]
    return runs[0] if runs else None


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


def download_results_artifact(repository, run_id, run_attempt, dest_dir):
    """只下载当前 attempt 的结果 artifact，拒绝混入旧目录内容。"""
    out = gh(
        "api",
        f"repos/{repository}/actions/runs/{run_id}/artifacts?per_page=100",
    )
    info = json.loads(out)
    artifacts = info.get("artifacts", [])
    expected_name = f"nav-results-{run_id}-{run_attempt}"
    matches = [item for item in artifacts if item.get("name") == expected_name]
    if len(matches) != 1:
        names = ", ".join(item.get("name", "?") for item in artifacts) or "<none>"
        print(f"[ERROR] 未找到唯一结果 artifact: {expected_name}")
        print(f"  当前 artifacts: {names}")
        sys.exit(1)

    dest_path = Path(dest_dir)
    if dest_path.exists() and any(dest_path.iterdir()):
        print(f"[ERROR] 目标目录非空，拒绝混入旧结果: {dest_path}")
        sys.exit(1)
    dest_path.mkdir(parents=True, exist_ok=True)
    gh(
        "run", "download", str(run_id),
        f"--name={expected_name}", f"--dir={dest_path}",
    )
    print(
        f"[fetch] 已下载 {expected_name} "
        f"({matches[0].get('size_in_bytes', '?')} bytes) 到 {dest_path}"
    )


def navigation_gitlink(repository, f1_commit):
    """从 GitHub commit tree 读取 navigation gitlink SHA。"""
    commit = json.loads(
        gh("api", f"repos/{repository}/git/commits/{f1_commit}")
    )
    tree_sha = commit.get("tree", {}).get("sha")
    if not tree_sha:
        print("[ERROR] 无法读取 F1 commit 的 tree SHA")
        sys.exit(1)
    tree = json.loads(gh("api", f"repos/{repository}/git/trees/{tree_sha}"))
    matches = [item for item in tree.get("tree", []) if item.get("path") == "navigation"]
    if len(matches) != 1 or matches[0].get("type") != "commit":
        print("[ERROR] F1 commit tree 中没有唯一的 navigation gitlink")
        sys.exit(1)
    return matches[0]["sha"]


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
    infrastructure_failure = None

    for f in sorted(Path(results_dir).rglob("*.json")):
        try:
            data = json.loads(f.read_text())
        except Exception:
            continue

        if f.name.startswith("batch_summary"):
            batch_summary = data
        elif f.name == "ci_diagnostic.json":
            ci_diag = data
        elif f.name == "infrastructure_failure.json":
            infrastructure_failure = data
        elif f.name == "evaluation_manifest.json":
            continue
        elif "metrics" in data:
            test_results.append(data)

    if not test_results and batch_summary:
        test_results = batch_summary

    if not test_results:
        report_lines.append("⚠ 未找到场景指标，评测基础设施未完成测试。")
        if infrastructure_failure:
            report_lines.append("")
            report_lines.append(
                f"- 原因: {infrastructure_failure.get('reason', 'unknown')}"
            )
            report_lines.append(
                f"- test outcome: {infrastructure_failure.get('test_outcome', 'unknown')}"
            )
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
    parser.add_argument("--run-id", type=int, help="指定 run ID")
    parser.add_argument(
        "--latest",
        action="store_true",
        help="显式允许使用最近一次结果；默认只匹配当前本地 F1 commit",
    )
    parser.add_argument("--sha", help="只匹配指定 F1 commit")
    parser.add_argument("--branch", type=str, help="指定分支")
    parser.add_argument("--timeout", type=int, default=2400, help="等待超时 (秒)")
    parser.add_argument("--dest", type=str, help="下载目录 (默认按 run/attempt 隔离)")
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

    repository = json.loads(gh("repo", "view", "--json=nameWithOwner"))["nameWithOwner"]

    # 1. 精确定位 run
    if args.run_id:
        run_id = args.run_id
        print(f"[fetch] 使用指定 run_id: {run_id}")
        run = json.loads(
            gh(
                "run", "view", str(run_id),
                "--json=attempt,status,conclusion,headBranch,headSha,displayTitle",
            )
        )
    else:
        expected_sha = args.sha
        if not expected_sha and not args.latest:
            local = subprocess.run(
                ["git", "rev-parse", "HEAD"],
                capture_output=True,
                text=True,
            )
            if local.returncode != 0:
                print("[ERROR] 无法读取本地 F1 commit；请传 --sha、--run-id 或 --latest")
                sys.exit(1)
            expected_sha = local.stdout.strip()
            print(f"[fetch] 默认匹配当前本地 F1 commit: {expected_sha}")

        run = find_latest_run(
            branch=args.branch,
            expected_sha=expected_sha,
        )
        if not run:
            suffix = f" for commit {expected_sha}" if expected_sha else ""
            print(f"[ERROR] 未找到 nav_eval workflow run{suffix}")
            print("  请先触发 CI: push 到 nav/dev-* 分支 或手动 dispatch")
            sys.exit(1)
        run_id = run["databaseId"]
        run = json.loads(
            gh(
                "run", "view", str(run_id),
                "--json=attempt,status,conclusion,headBranch,headSha,displayTitle",
            )
        )

    print(
        f"[fetch] run={run_id} attempt={run['attempt']} "
        f"status={run.get('status')} branch={run.get('headBranch', '?')}"
    )
    print(f"  F1 commit: {run.get('headSha')}")
    print(f"  title: {run.get('displayTitle', '?')}")

    # 2. 等待完成
    if not args.no_wait:
        conclusion = wait_for_run(run_id, timeout=args.timeout)
    else:
        out = gh("run", "view", str(run_id), "--json=status,conclusion")
        conclusion = json.loads(out).get("conclusion", "unknown")

    # 3. 下载当前 attempt 的唯一结果 artifact
    dest = os.path.abspath(
        args.dest or os.path.join("ci_fetched", f"{run_id}-{run['attempt']}")
    )
    download_results_artifact(repository, run_id, run["attempt"], dest)

    # 4. 校验 run → F1 commit → navigation gitlink → 文件 hash
    expected_navigation = navigation_gitlink(repository, run["headSha"])
    try:
        manifest = verify_manifest(
            Path(dest),
            expected_run_id=str(run_id),
            expected_run_attempt=run["attempt"],
            expected_f1_commit=run["headSha"],
            expected_navigation_commit=expected_navigation,
        )
    except (ManifestError, OSError, ValueError, json.JSONDecodeError) as exc:
        print(f"[ERROR] 结果证据校验失败: {exc}")
        sys.exit(1)
    print(
        "[fetch] 证据校验通过: "
        f"F1={run['headSha']} navigation={expected_navigation} "
        f"attempt={manifest['run']['run_attempt']}"
    )

    # 5. 分析
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
