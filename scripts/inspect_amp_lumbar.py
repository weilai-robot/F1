#!/usr/bin/env python3
"""Read existing AMP walk_diag CSVs; never publish robot commands."""

import argparse
import csv
import math
from pathlib import Path
import statistics

JOINT = "lumbar_pitch_joint"
FIELDS = {
    "raw": f"pos_des_raw_{JOINT}",
    "target": f"pos_des_lpf_{JOINT}",
    "position": f"pos_{JOINT}",
    "base_pitch": "base_euler_y",
}


def latest_log():
    root = Path(__file__).resolve().parent.parent
    candidates = []
    for directory in [Path.cwd(), root / "build", root / "motion_control"]:
        candidates.extend((directory / "test_logs/data_csv").glob("walk_diag_*.csv"))
    if not candidates:
        raise ValueError("未找到 walk_diag CSV，请显式传入 AMP 日志路径。")
    return max(candidates, key=lambda path: path.stat().st_mtime_ns)


def summarize(path, skip_seconds):
    samples = []
    with path.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        required = {"timestamp_ns", *FIELDS.values()}
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise ValueError("缺少 AMP 全身日志列：" + ", ".join(sorted(missing)))
        first_ns = None
        for row in reader:
            # A running logging worker may still be writing the final row.
            try:
                timestamp = int(row["timestamp_ns"])
                values = {key: float(row[column]) for key, column in FIELDS.items()}
            except (ValueError, TypeError):
                continue
            if not all(math.isfinite(value) for value in values.values()):
                continue
            if first_ns is None:
                first_ns = timestamp
            if (timestamp - first_ns) / 1e9 < skip_seconds:
                continue
            samples.append(values)
    if not samples:
        raise ValueError("没有有效样本；可用 --skip-seconds 0 检查短日志。")
    return samples


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="?", type=Path, help="默认选择最新 walk_diag CSV")
    parser.add_argument("--skip-seconds", type=float, default=2.0,
                        help="略过日志开头的切入过程，默认 2 秒")
    args = parser.parse_args()
    if not math.isfinite(args.skip_seconds) or args.skip_seconds < 0:
        parser.error("--skip-seconds 必须是非负有限值")
    try:
        path = args.csv if args.csv is not None else latest_log()
        samples = summarize(path, args.skip_seconds)
    except (OSError, ValueError, csv.Error) as error:
        parser.error(str(error))

    print(f"日志：{path.resolve()}\n有效样本：{len(samples)}")
    print("角度：均值 [最小值, 最大值]；rad / deg")
    labels = {"raw": "策略原始腰目标", "target": "限位/滤波后腰目标",
              "position": "策略使用的腰反馈", "base_pitch": "IMU base pitch"}
    for key, label in labels.items():
        values = [sample[key] for sample in samples]
        avg, lo, hi = statistics.fmean(values), min(values), max(values)
        print(f"{label}: {avg:+.5f} [{lo:+.5f}, {hi:+.5f}] / "
              f"{math.degrees(avg):+.2f}° [{math.degrees(lo):+.2f}°, {math.degrees(hi):+.2f}°]")
    altered = sum(abs(sample["raw"] - sample["target"]) > 1e-5 for sample in samples)
    errors = [sample["target"] - sample["position"] for sample in samples]
    mae = statistics.fmean(abs(error) for error in errors)
    print(f"原始与最终目标不同：{altered}/{len(samples)} 帧（{altered / len(samples):.1%}）")
    print(f"腰角跟踪 MAE：{mae:.5f} rad / {math.degrees(mae):.2f}°")
    print("模型约定：腰 pitch 增加表示向前折；IMU pitch 需先确认安装坐标。")
    print("日志只能验证数值跟踪；需观察实际向前/向后变化才能确认机械方向和零位。")


if __name__ == "__main__":
    main()
