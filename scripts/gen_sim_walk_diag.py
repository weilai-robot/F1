#!/usr/bin/env python3
"""Generate a deploy-format walk_diag CSV from a sim2sim rollout npz.

Purpose: an apples-to-apples SIM BASELINE in the exact schema the robot logs
(doc/amp_deploy_v70b_20261015.zh_CN.md), so scripts/analyze_amp_walk_logs.py
can diff real walk_diag_*.csv against the sim without format glue.

Method: causal policy replay — obs at t uses logged q/dq/quat up to t and the
replayed action at t-1, which reproduces the original closed-loop action
sequence (same policy, same states). Targets/torques use the v68 PD table read
from the deployed rl_x1.yaml (policy order), matching the real-robot path.

JOINT ORDER (v2 fix): the rollout npz stores q/dq in MuJoCo HINGE order
(serial: lumbar, arms, legs — npz meta.hinge_names), but the policy obs/action
terms are in LAB order (interleaved — retarget config lab_dof_names, same as
rl_walk_amp.joint_list). Exactly like sim2sim mujoco_rollout.py:
  obs jpos/jvel = (q - q_default)[lab2hinge]      # lab2hinge[i] = hinge idx of lab[i]
  q_target[lab2hinge] = q_default[lab2hinge] + 0.25 * act
CSV columns are written BY NAME, so the emitted file matches the robot schema
regardless of internal order.

Synthetic/approximate fields (documented, NOT measurements):
  effort_<j>    = tau_des (commanded PD torque; sim qfrc not logged)
  imu_gyro_*    = finite-difference body angular velocity from base_quat
  imu_accel_*   = finite-difference of body linear velocity (v_b)
  actuator_*    = omitted (no motor-side channels in sim)
  phase_*       = 0/1 (AMP obs has no phase term)
  clip_count    = 0

Usage:
  python3 scripts/gen_sim_walk_diag.py <rollout.npz> <policy.npz> <out.csv> \
      [--config motion_control/module/control_module/cfg/rl_x1.yaml]
"""
from __future__ import annotations
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def load_policy(npz_path):
    z = np.load(npz_path, allow_pickle=False)
    layers, i = [], 0
    while f"l{i}_w" in z.files:
        layers.append((z[f"l{i}_w"].astype(np.float32),
                       z[f"l{i}_b"].astype(np.float32) if f"l{i}_b" in z.files else None))
        i += 1
    mean, std = z["mean"].astype(np.float32), z["std"].astype(np.float32)

    def run(obs):  # obs: (288,) raw (normalizer fused here)
        x = (obs - mean) / std
        for w, b in layers[:-1]:
            x = x @ w.T + (b if b is not None else 0)
            x = np.where(x < 0, np.expm1(np.minimum(x, 20)), x)  # ELU
        w, b = layers[-1]
        return x @ w.T + (b if b is not None else 0)

    return run


def quat_to_R(bq):
    w, x, y, z = bq / np.linalg.norm(bq)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def euler_xyz(R):
    pitch = math.asin(max(-1.0, min(1.0, -R[0, 2])))
    roll = math.atan2(R[1, 2], R[2, 2])
    yaw = math.atan2(R[0, 1], R[0, 0])
    return roll, pitch, yaw


def parse_joint_list(config_path):
    """rl_walk_amp.joint_list (lab/policy order) -> [names]."""
    text = Path(config_path).read_text()
    import re
    sec = re.search(r"^  rl_walk_amp:\n(.*?)(?=^  \w|\Z)", text, re.M | re.S).group(1)
    jl = re.search(r"joint_list:\s*\[([^\]]*)\]", sec, re.M).group(1)
    names = [x.strip() for x in jl.replace("\n", " ").split(",")]
    assert len(names) == 29, f"joint_list has {len(names)} entries"
    return names


def parse_pd(config_path, joint_list):
    """stiffness/damping arrays in rl_walk_amp (policy order) -> dict by name."""
    text = Path(config_path).read_text()
    import re
    sec = re.search(r"^  rl_walk_amp:\n(.*?)(?=^  \w|\Z)", text, re.M | re.S).group(1)
    kp = [float(v) for v in re.search(r"^\s*stiffness:\s*\[([^\]]*)\]", sec, re.M).group(1).split(",")]
    kd = [float(v) for v in re.search(r"^\s*damping:\s*\[([^\]]*)\]", sec, re.M).group(1).split(",")]
    init = [float(v) for v in re.search(r"^\s*init_state:\s*\[([^\]]*)\]", sec, re.M).group(1).split(",")]
    assert len(kp) == len(joint_list) == len(kd) == len(init), "PD array length mismatch"
    return ({n: (kp[i], kd[i]) for i, n in enumerate(joint_list)},
            {n: init[i] for i, n in enumerate(joint_list)})


SLICES = [(0, 3), (3, 6), (6, 9), (9, 38), (38, 67), (67, 96)]
PARALLEL = {"left_ankle_pitch_joint", "left_ankle_roll_joint",
            "right_ankle_pitch_joint", "right_ankle_roll_joint"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rollout")
    ap.add_argument("policy")
    ap.add_argument("out")
    ap.add_argument("--config", default=str(ROOT / "motion_control/module/control_module/cfg/rl_x1.yaml"))
    ap.add_argument("--settle-s", type=float, default=0.5,
                    help="drop first N s of the rollout (policy warm-up)")
    args = ap.parse_args()

    z = np.load(args.rollout, allow_pickle=False)
    meta = json.loads(str(z["meta"]))
    hinge = list(meta["hinge_names"])
    cmd = np.asarray(meta["cmd"], dtype=np.float32)
    cdt = float(meta.get("control_dt", 0.02))
    policy = load_policy(args.policy)

    # lab (policy) order from the deployed config; hinge->lab permutation,
    # exactly like sim2sim mujoco_rollout.py (obs/action are LAB order)
    names = parse_joint_list(args.config)
    assert sorted(names) == sorted(hinge), "joint name sets differ (config vs npz)"
    lab2hinge = np.array([hinge.index(n) for n in names])

    pd_map, q_def_map = parse_pd(args.config, names)

    q, dq, bq, v_b = z["q"], z["dq"], z["base_quat"], z["v_b"]
    q_lab = q[:, lab2hinge]
    dq_lab = dq[:, lab2hinge]
    q_def = np.array([q_def_map[n] for n in names], dtype=np.float64)
    kp = np.array([pd_map[n][0] for n in names])
    kd = np.array([pd_map[n][1] for n in names])

    # causal replay -> actions (lab order)
    hist = np.zeros((3, 96), dtype=np.float32)
    last = np.zeros(29, dtype=np.float32)
    actions, gravs, gyros = [], [], []
    for t in range(len(q)):
        R = quat_to_R(bq[t])
        grav = R.T @ np.array([0, 0, -1.0])
        if t > 0:
            d = (bq[t] - bq[t - 1]) / cdt
            w, x, y, zz = bq[t] / np.linalg.norm(bq[t])
            ang = 2.0 * np.array([w * d[1] - x * d[0] - y * d[3] + zz * d[2],
                                  w * d[2] + x * d[3] - y * d[0] - zz * d[1],
                                  w * d[3] - x * d[2] + y * d[1] - zz * d[0]])
        else:
            ang = np.zeros(3)
        frame = np.concatenate([ang, grav, cmd, (q_lab[t] - q_def).astype(np.float32),
                                dq_lab[t].astype(np.float32), last]).astype(np.float32)
        hist[:-1] = hist[1:]
        hist[-1] = frame
        obs = np.concatenate([hist[:, a:b].reshape(-1) for a, b in SLICES])
        a = policy(obs).astype(np.float32)
        actions.append(a)
        gravs.append(grav)
        gyros.append(ang)
        last = a
    actions = np.stack(actions)
    gravs = np.stack(gravs)
    gyros = np.stack(gyros)

    acc = np.vstack([np.zeros(3), np.diff(v_b, axis=0) / cdt])  # approx body accel
    pos_des = q_def[None, :] + 0.25 * actions              # lab order
    tau_des = kp[None, :] * (pos_des - q_lab) + kd[None, :] * (0.0 - dq_lab)

    t0 = int(args.settle_s / cdt)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="") as f:
        w = csv.writer(f)
        head = ["timestamp_ns", "phase_sin", "phase_cos",
                "cmd_linear_x", "cmd_linear_y", "cmd_angular_z",
                "base_euler_x", "base_euler_y", "base_euler_z",
                "base_ang_vel_x", "base_ang_vel_y", "base_ang_vel_z"]
        for n in names:
            head += [f"action_{n}", f"pos_{n}", f"vel_{n}", f"effort_{n}",
                     f"pos_des_raw_{n}", f"pos_des_lpf_{n}",
                     f"tau_des_raw_{n}", f"tau_des_lpf_{n}", f"is_parallel_{n}"]
        head += ["clip_count", "imu_quat_w", "imu_quat_x", "imu_quat_y", "imu_quat_z",
                 "imu_gyro_x", "imu_gyro_y", "imu_gyro_z",
                 "imu_accel_x", "imu_accel_y", "imu_accel_z"]
        w.writerow(head)
        for t in range(t0, len(q)):
            R = quat_to_R(bq[t])
            roll, pitch, yaw = euler_xyz(R)
            row = [int(round(t * cdt * 1e9)), 0.0, 1.0,
                   float(cmd[0]), float(cmd[1]), float(cmd[2]),
                   roll, pitch, yaw,
                   gyros[t, 0], gyros[t, 1], gyros[t, 2]]
            for i, n in enumerate(names):
                row += [float(actions[t, i]), float(q_lab[t, i]), float(dq_lab[t, i]), float(tau_des[t, i]),
                        float(pos_des[t, i]), float(pos_des[t, i]),
                        float(tau_des[t, i]), float(tau_des[t, i]),
                        int(n in PARALLEL)]
            qn = bq[t] / np.linalg.norm(bq[t])
            row += [0, qn[0], qn[1], qn[2], qn[3],
                    gyros[t, 0], gyros[t, 1], gyros[t, 2],
                    acc[t, 0], acc[t, 1], acc[t, 2]]
            w.writerow(row)
    print(f"[OK] {out}: {len(q) - t0} rows (settle {args.settle_s}s dropped), "
          f"source={args.rollout}, policy={args.policy}")


if __name__ == "__main__":
    main()
