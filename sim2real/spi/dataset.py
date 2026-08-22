"""Real-robot log -> SPI clip dataset.

Parses the 100 Hz data_file_logger CSVs produced by motion_control
(``czy/real_data/round_exp_*.csv`` excitation experiments, ``czy/8.7/walk_diag_*.csv``
walking diagnostics) into variable-horizon clips following the paper's
"simulation error criterion" segmentation: H ~ U(H_min, H_max) seconds.

Log columns (per leg joint ``<j>``):
    action_<j>, pos_<j>, vel_<j>, effort_<j>,
    pos_des_raw_<j>, pos_des_lpf_<j>, tau_des_raw_<j>, tau_des_lpf_<j>,
    is_parallel_<j>
Global: timestamp_ns, phase_sin/cos, cmd_*, [left/right_contact],
        base_euler_*, base_ang_vel_*, clip_count,
        imu_quat_{w,x,y,z}, imu_gyro_*, imu_accel_*

Drive model on X1 (rl_controller.cc:28):
  * hip / knee  (serial motors):   position command -> driver PD -> torque
        tau_PD = kp (q_des - q) - kd qdot        [kp, kd from experiment cfg]
  * ankle pitch/roll (parallel):   direct effort command tau_des_lpf
During replay the *simulated* q, qdot are used inside tau_PD (true open-loop
prediction); the tanh saturation model is applied in spi.rollout.

Output: single .npz with stacked clips + metadata json (see ``ClipDataset``).
"""

from __future__ import annotations

import csv
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

LEG_JOINTS = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_pitch_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_pitch_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
]
PARALLEL_JOINTS = {
    "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_ankle_pitch_joint", "right_ankle_roll_joint",
}
# Non-logged joints held at nominal standing pose (pd_stand init_state in rl_x1.yaml)
HOLD_POSE = {
    "lumbar_yaw_joint": 0.0, "lumbar_roll_joint": 0.0, "lumbar_pitch_joint": 0.0,
    "left_shoulder_pitch_joint": 0.15, "left_shoulder_roll_joint": -0.1,
    "left_shoulder_yaw_joint": 0.0, "left_elbow_pitch_joint": 0.3,
    "left_elbow_yaw_joint": 0.0, "left_wrist_pitch_joint": 0.0,
    "left_wrist_roll_joint": 0.0,
    "right_shoulder_pitch_joint": 0.15, "right_shoulder_roll_joint": -0.1,
    "right_shoulder_yaw_joint": 0.0, "right_elbow_pitch_joint": 0.3,
    "right_elbow_yaw_joint": 0.0, "right_wrist_pitch_joint": 0.0,
    "right_wrist_roll_joint": 0.0,
}
HOLD_KP, HOLD_KD = 80.0, 1.5  # arm-hold PD (rl_x1.yaml arm gains)

MODE_POS, MODE_TAU = 0, 1


@dataclass
class RobotLog:
    """One parsed CSV in memory."""
    t: np.ndarray                 # (N,) seconds from first sample
    dt: float                     # median sample period [s]
    imu_quat: np.ndarray          # (N,4) w,x,y,z
    imu_gyro: np.ndarray          # (N,3) rad/s (body frame)
    imu_accel: np.ndarray         # (N,3) specific force m/s^2 (body frame)
    q: np.ndarray                 # (N,29) joint pos, LEG_JOINTS+HOLD order? -> full 29 (rl_x1 order)
    qd: np.ndarray                # (N,29)
    tau_meas: np.ndarray          # (N,29) measured effort (nan for unlogged)
    target_pos: np.ndarray        # (N,29) pos_des_lpf (nan for parallel/hold)
    target_tau: np.ndarray        # (N,29) tau_des_lpf (nan for serial/hold)
    mode: np.ndarray              # (N,29) int8 MODE_POS / MODE_TAU / -1 hold (unlogged)
    cmd: np.ndarray               # (N,3) vx, vy, wz
    clip_id: np.ndarray           # (N,) int
    kp: np.ndarray                # (29,)
    kd: np.ndarray                # (29,)

    @property
    def joint_names(self) -> List[str]:
        return FULL_JOINT_ORDER


FULL_JOINT_ORDER = [
    "lumbar_yaw_joint", "lumbar_roll_joint", "lumbar_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_pitch_joint", "left_elbow_yaw_joint", "left_wrist_pitch_joint",
    "left_wrist_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_pitch_joint", "right_elbow_yaw_joint", "right_wrist_pitch_joint",
    "right_wrist_roll_joint",
    *LEG_JOINTS,
]
JIDX = {n: i for i, n in enumerate(FULL_JOINT_ORDER)}


def parse_csv(path: str | Path, kp: Dict[str, float], kd: Dict[str, float],
              base_height: float = 0.72) -> RobotLog:
    """Parse one logger CSV.

    kp/kd: per-leg-joint PD gains used in that experiment (serial joints).
    Parallel ankle joints ignore kp/kd (torque command).
    """
    path = Path(path)
    with open(path) as f:
        reader = csv.reader(f)
        header = next(reader)
        rows = [r for r in reader if r]
    col = {name: i for i, name in enumerate(header)}

    def getf(row, name, default=np.nan) -> float:
        i = col.get(name)
        if i is None:
            return default
        v = row[i]
        try:
            return float(v)
        except ValueError:
            return default  # 'nan' or empty

    n = len(rows)
    ts = np.array([int(r[col["timestamp_ns"]]) for r in rows], dtype=np.int64)
    t = (ts - ts[0]) * 1e-9
    dt = float(np.median(np.diff(t)))

    imu_quat = np.array([[getf(r, f"imu_quat_{c}") for c in "wxyz"] for r in rows])
    imu_gyro = np.array([[getf(r, f"imu_gyro_{c}") for c in "xyz"] for r in rows])
    imu_accel = np.array([[getf(r, f"imu_accel_{c}") for c in "xyz"] for r in rows])
    cmd = np.array([[getf(r, f"cmd_linear_x"), getf(r, f"cmd_linear_y"),
                     getf(r, f"cmd_angular_z")] for r in rows])

    q = np.full((n, 29), np.nan)
    qd = np.full((n, 29), np.nan)
    tau_meas = np.full((n, 29), np.nan)
    target_pos = np.full((n, 29), np.nan)
    target_tau = np.full((n, 29), np.nan)
    mode = np.full((n, 29), -1, dtype=np.int8)
    for jn in LEG_JOINTS:
        j = JIDX[jn]
        q[:, j] = [getf(r, f"pos_{jn}") for r in rows]
        qd[:, j] = [getf(r, f"vel_{jn}") for r in rows]
        tau_meas[:, j] = [getf(r, f"effort_{jn}") for r in rows]
        target_pos[:, j] = [getf(r, f"pos_des_lpf_{jn}") for r in rows]
        target_tau[:, j] = [getf(r, f"tau_des_lpf_{jn}") for r in rows]
        is_par = int(float(rows[0][col[f"is_parallel_{jn}"]]))
        mode[:, j] = MODE_TAU if is_par else MODE_POS
    # unlogged upper body: hold at nominal pose
    for jn, pose in HOLD_POSE.items():
        j = JIDX[jn]
        q[:, j] = pose
        qd[:, j] = 0.0
        tau_meas[:, j] = np.nan
        target_pos[:, j] = pose
        mode[:, j] = MODE_POS

    clip_id = np.zeros(n, dtype=np.int64)
    if "clip_count" in col:
        clip_id = np.array([int(float(r[col["clip_count"]])) for r in rows])

    kp_v = np.full(29, HOLD_KP)
    kd_v = np.full(29, HOLD_KD)
    for jn, v in kp.items():
        kp_v[JIDX[jn]] = v
    for jn, v in kd.items():
        kd_v[JIDX[jn]] = v

    return RobotLog(t=t, dt=dt, imu_quat=imu_quat, imu_gyro=imu_gyro,
                    imu_accel=imu_accel, q=q, qd=qd,
                    tau_meas=tau_meas, target_pos=target_pos, target_tau=target_tau,
                    mode=mode, cmd=cmd, clip_id=clip_id, kp=kp_v, kd=kd_v)


def segment_clips(log: RobotLog, h_min_s: float = 1.0, h_max_s: float = 2.0,
                  seed: int = 0) -> List[Dict]:
    """Cut the log into clips of horizon H ~ U(H_min, H_max) seconds.

    Splits also on clip_id discontinuities (episode markers). Non-finite
    reference rows at clip boundaries are dropped. Each clip dict:
      {q0, qd0, quat0, gyro0, ctrl_target_pos, ctrl_target_tau, mode,
       ref_quat, ref_gyro, ref_accel, ref_q, ref_qd, ref_tau, n, dt, kp, kd}
    """
    rng = np.random.default_rng(seed)
    clips: List[Dict] = []
    bounds = [0]
    dclip = np.where(np.diff(log.clip_id) != 0)[0] + 1
    bounds.extend(dclip.tolist())
    bounds.append(len(log.t))

    for s, e in zip(bounds[:-1], bounds[1:]):
        seg = e - s
        pos = 0
        while pos < seg - 5:  # need at least 5 samples
            h_s = float(rng.uniform(h_min_s, h_max_s))
            h = max(5, int(round(h_s / log.dt)))
            idx = np.arange(s + pos, min(s + pos + h, e))
            # skip clips containing nan in reference joint states
            if np.isnan(log.q[idx][:, 17:]).any() or np.isnan(log.qd[idx][:, 17:]).any():
                pos += h
                continue
            clip = dict(
                q0=log.q[idx[0]].copy(),
                qd0=log.qd[idx[0]].copy(),
                quat0=log.imu_quat[idx[0]].copy(),
                gyro0=log.imu_gyro[idx[0]].copy(),
                ctrl_target_pos=log.target_pos[idx].copy(),
                ctrl_target_tau=log.target_tau[idx].copy(),
                mode=log.mode[idx].copy(),
                ref_quat=log.imu_quat[idx].copy(),
                ref_gyro=log.imu_gyro[idx].copy(),
                ref_accel=log.imu_accel[idx].copy(),
                ref_q=log.q[idx].copy(),
                ref_qd=log.qd[idx].copy(),
                ref_tau=log.tau_meas[idx].copy(),
                cmd=log.cmd[idx].copy(),
                n=len(idx), dt=log.dt,
                kp=log.kp.copy(), kd=log.kd.copy(),
            )
            clips.append(clip)
            pos += h
    return clips


def save_clips(clips: List[Dict], meta: Dict, out_path: str | Path) -> None:
    """Stack clips (padded with the last frame) into one npz + embedded meta."""
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    n_max = max(c["n"] for c in clips)
    keys = ["ctrl_target_pos", "ctrl_target_tau", "mode", "ref_quat", "ref_gyro",
            "ref_accel", "ref_q", "ref_qd", "ref_tau", "cmd"]

    def pad(arr: np.ndarray, n_max: int) -> np.ndarray:
        if arr.shape[0] == n_max:
            return arr
        pad_idx = np.arange(arr.shape[0], n_max)
        return np.concatenate([arr, np.repeat(arr[-1:], n_max - arr.shape[0], axis=0)], axis=0)

    payload = {
        "n_max": n_max,
        "lengths": np.array([c["n"] for c in clips], dtype=np.int32),
        "q0": np.stack([c["q0"] for c in clips]),
        "qd0": np.stack([c["qd0"] for c in clips]),
        "quat0": np.stack([c["quat0"] for c in clips]),
        "gyro0": np.stack([c["gyro0"] for c in clips]),
        "kp": np.stack([c["kp"] for c in clips]),
        "kd": np.stack([c["kd"] for c in clips]),
        "dt": np.array([c["dt"] for c in clips]),
        "meta_json": json.dumps({**meta, "joint_order": FULL_JOINT_ORDER}),
    }
    for k in keys:
        payload[k] = np.stack([pad(c[k], n_max) for c in clips])
    np.savez_compressed(out_path, **payload)


def load_clips(path: str | Path) -> tuple[List[Dict], Dict]:
    """Inverse of save_clips."""
    z = np.load(Path(path), allow_pickle=False)
    meta = json.loads(str(z["meta_json"]))
    lengths = z["lengths"]
    clips = []
    for i, n in enumerate(lengths):
        sl = slice(0, int(n))
        clip = dict(
            q0=z["q0"][i], qd0=z["qd0"][i], quat0=z["quat0"][i], gyro0=z["gyro0"][i],
            n=int(n), dt=float(z["dt"][i]), kp=z["kp"][i], kd=z["kd"][i],
        )
        for k in ["ctrl_target_pos", "ctrl_target_tau", "mode", "ref_quat",
                  "ref_gyro", "ref_accel", "ref_q", "ref_qd", "ref_tau", "cmd"]:
            clip[k] = z[k][i][sl]
        clips.append(clip)
    return clips, meta
