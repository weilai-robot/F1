#!/usr/bin/env python3
"""Sanity check: IMU accel convention vs R^T * g (specific force)."""
import csv
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def load(path, n=300):
    with open(path) as f:
        r = csv.reader(f)
        header = next(r)
        rows = [row for row in r][:n]
    col = {name: i for i, name in enumerate(header)}

    def getf(row, name):
        i = col.get(name)
        return float(row[i]) if i is not None else np.nan

    q = np.array([[getf(r, f"imu_quat_{c}") for c in "wxyz"] for r in rows])
    a = np.array([[getf(r, f"imu_accel_{c}") for c in "xyz"] for r in rows])
    return q, a


def quat_rotate_inv(q, v):
    q = q / np.linalg.norm(q)
    w, x, y, z = q
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                  [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                  [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    return R @ v


def main() -> None:
    G = np.array([0.0, 0.0, 9.81])
    for path in [
        "motion_control/czy/8.7/walk_diag_20260807_093740.csv",
        "motion_control/czy/real_data/round_exp_010_kp40_kd3_ff0_20260804_105929.csv",
    ]:
        p = ROOT / path
        if not p.exists():
            print(f"skip {path}: missing")
            continue
        q, a = load(p)
        pred = np.array([quat_rotate_inv(qq, G) for qq in q])
        print(f"[{path.split('/')[-1]}]")
        print("  quat0(wxyz):", np.round(q[0], 4))
        print("  measured accel mean:", np.round(a.mean(axis=0), 3))
        print("  predicted R^T*g mean:", np.round(pred.mean(axis=0), 3))
        print("  per-axis corr:", [round(float(np.corrcoef(a[:, i], pred[:, i])[0, 1]), 3)
                                   for i in range(3)])


if __name__ == "__main__":
    sys.exit(main())
