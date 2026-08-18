"""MuJoCo open-loop replay of recorded controls for SPI parameter scoring.

For each clip:
  * initial state aligned with the real trajectory (joints from log; base
    quaternion from IMU; base height solved so the lowest foot touches ground;
    base angular velocity from gyro, expressed in the body frame — MuJoCo free
    joint convention)
  * recorded controls are replayed through the X1 drive chain:
      serial joints (hip/knee):  tau_PD = kp (q_des - q) - kd qdot  [sim state]
      parallel ankles:           tau    = tau_des_lpf (log)
    then the tanh actuator model  tau_m = kappa_s * kappa * tanh(tau_PD / kappa)
  * logs simulated quat / gyro / q / qd / tau at the data-sample rate.

All mujoco imports are lazy: this module is only imported when actually
running sysid (remote image or any machine with mujoco installed), so the
numpy-only unit tests never require it.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

from .dataset import FULL_JOINT_ORDER, MODE_POS, MODE_TAU
from .param_space import tanh_motor_torque


def _quat_wxyz_to_xyzw(q):
    return np.array([q[1], q[2], q[3], q[0]])


def mat2quat_wxyz(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> unit quaternion (w, x, y, z)."""
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


class MuJoCoRollouter:
    def __init__(self, mjcf_path: str | Path, base_body: str = "link_base",
                 foot_bodies: Tuple[str, ...] = ("link_left_ankle_roll", "link_right_ankle_roll"),
                 gyro_in_body_frame: bool = True):
        import mujoco  # lazy

        self._mj = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(mjcf_path))
        self.data = mujoco.MjData(self.model)
        self.base_bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, base_body)
        if self.base_bid < 0:
            raise ValueError(f"body '{base_body}' not found in {mjcf_path}")
        self.foot_bids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, f)
                          for f in foot_bodies]
        self.gyro_in_body_frame = gyro_in_body_frame

        # joint address maps for the 29 logged joints
        self.j_qpos_adr, self.j_dof_adr, self.j_ctrl_adr = [], [], []
        missing = []
        for jn in FULL_JOINT_ORDER:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jn)
            if jid < 0:
                missing.append(jn)
                self.j_qpos_adr.append(None)
                self.j_dof_adr.append(None)
                self.j_ctrl_adr.append(None)
                continue
            self.j_qpos_adr.append(self.model.jnt_qposadr[jid])
            self.j_dof_adr.append(self.model.jnt_dofadr[jid])
            # MJCF actuator names drop the "_joint" suffix of joint names
            stem = jn[:-6] if jn.endswith("_joint") else jn
            act = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"motor_{jn}")
            if act < 0:
                act = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"motor_{stem}")
            self.j_ctrl_adr.append(act if act >= 0 else None)
        if missing:
            raise ValueError(f"joints missing from MJCF: {missing}")
        # free joint of base body
        base_jid = self.model.body_jntadr[self.base_bid]
        self.free_qposadr = self.model.jnt_qposadr[base_jid]
        self.free_dofadr = self.model.jnt_dofadr[base_jid]
        self.nq, self.nv, self.nu = self.model.nq, self.model.nv, self.model.nu

        # nominal inertial values (for reset)
        self._nominal = {
            "mass": float(self.model.body_mass[self.base_bid]),
            "ipos": self.model.body_ipos[self.base_bid].copy(),
            "inertia": self.model.body_inertia[self.base_bid].copy(),
            "iquat": self.model.body_iquat[self.base_bid].copy(),
        }

    # ------------------------------------------------------------------
    def set_body_params(self, params: Dict) -> None:
        """Apply identified (m, r, I_full) of the base body to the model."""
        mj = self._mj
        p = params["bodies"].get(self._base_name)
        if p is None:
            return
        self.model.body_mass[self.base_bid] = p["mass"]
        self.model.body_ipos[self.base_bid] = p["com"]
        I = np.asarray(p["inertia"], dtype=float)
        I = 0.5 * (I + I.T)
        lam, V = np.linalg.eigh(I)          # I = V diag(lam) V^T, V columns axes
        if np.linalg.det(V) < 0:            # ensure right-handed frame
            V[:, 0] = -V[:, 0]
        self.model.body_inertia[self.base_bid] = np.maximum(lam, 1e-9)
        self.model.body_iquat[self.base_bid] = mat2quat_wxyz(V)
        mj.mj_setConst(self.model, self.data)   # recompute derived quantities

    def reset_body_params(self) -> None:
        self.model.body_mass[self.base_bid] = self._nominal["mass"]
        self.model.body_ipos[self.base_bid] = self._nominal["ipos"]
        self.model.body_inertia[self.base_bid] = self._nominal["inertia"]
        self.model.body_iquat[self.base_bid] = self._nominal["iquat"]
        self._mj.mj_setConst(self.model, self.data)

    _base_name = "base"  # key into params["bodies"] (mapped from config)

    # ------------------------------------------------------------------
    def _initial_state(self, clip: Dict) -> Tuple[np.ndarray, np.ndarray]:
        mj = self._mj
        qpos = np.zeros(self.nq)
        qvel = np.zeros(self.nv)
        # base: position solved so lowest foot geom z == 0
        qpos[self.free_qposadr + 3:self.free_qposadr + 7] = clip["quat0"]
        for i, adr in enumerate(self.j_qpos_adr):
            qpos[adr] = clip["q0"][i]
        qpos[self.free_qposadr + 2] = 1.0  # provisional height
        mj.mj_kinematics(self.model, self.data)
        mj.mj_comPos(self.model, self.data)
        self.data.qpos[:] = qpos
        mj.mj_kinematics(self.model, self.data)
        z_min = min(self.data.xpos[bid][2] for bid in self.foot_bids if bid >= 0)
        qpos[self.free_qposadr + 2] -= z_min
        # velocities: base linear unknown -> 0; angular from gyro (body frame)
        qvel[self.free_dofadr + 3:self.free_dofadr + 6] = clip["gyro0"]
        for i, adr in enumerate(self.j_dof_adr):
            qvel[adr] = clip["qd0"][i]
        return qpos, qvel

    def _motor_torques(self, clip: Dict, row: int, qpos: np.ndarray,
                       qvel: np.ndarray, kappa_j: np.ndarray,
                       kappa_s: float) -> np.ndarray:
        """tau for the 29 joints with the tanh actuator model."""
        tau = np.zeros(29)
        for i in range(29):
            mode = clip["mode"][row, i]
            kappa = kappa_j[i]
            if mode == MODE_POS:
                tau_pd = clip["kp"][i] * (clip["ctrl_target_pos"][row, i] - qpos[self.j_qpos_adr[i]]) \
                    - clip["kd"][i] * qvel[self.j_dof_adr[i]]
            elif mode == MODE_TAU:
                tau_pd = clip["ctrl_target_tau"][row, i]
            else:  # hold at nominal (shoulder/wrist/lumbar)
                tau_pd = clip["kp"][i] * (clip["ctrl_target_pos"][row, i] - qpos[self.j_qpos_adr[i]]) \
                    - clip["kd"][i] * qvel[self.j_dof_adr[i]]
            tau[i] = tanh_motor_torque(tau_pd, kappa, kappa_s)
        return tau

    def rollout_clip(self, clip: Dict, params: Dict,
                     kappa_map: Dict[str, List[int]]) -> Dict[str, np.ndarray]:
        """Open-loop rollout of one clip under candidate params."""
        mj = self._mj
        self.set_body_params(params)
        kappa_s = params.get("kappa_s", 1.0)
        kappa_j = np.full(29, 1e9)  # default: effectively no saturation
        for gname, idxs in kappa_map.items():
            kappa_j[idxs] = params["motors"].get(gname, kappa_j[idxs[0]])

        qpos, qvel = self._initial_state(clip)
        self.data.qpos[:] = qpos
        self.data.qvel[:] = qvel
        mj.mj_resetData(self.model, self.data)
        self.data.qpos[:] = qpos
        self.data.qvel[:] = qvel

        n = clip["n"]
        sim_steps = max(1, int(round(clip["dt"] / self.model.opt.timestep)))
        out = {"quat": np.zeros((n, 4)), "gyro": np.zeros((n, 3)),
               "q": np.zeros((n, 29)), "qd": np.zeros((n, 29)), "tau": np.zeros((n, 29))}

        for row in range(n):
            tau = self._motor_torques(clip, row, self.data.qpos, self.data.qvel,
                                      kappa_j, kappa_s)
            ctrl = np.zeros(self.nu)
            for i in range(29):
                if self.j_ctrl_adr[i] is not None:
                    ctrl[self.j_ctrl_adr[i]] = tau[i]
            for _ in range(sim_steps):
                self.data.ctrl[:] = ctrl
                mj.mj_step(self.model, self.data)
            fq = self.free_qposadr
            fd = self.free_dofadr
            out["quat"][row] = self.data.qpos[fq + 3:fq + 7]
            out["gyro"][row] = self.data.qvel[fd + 3:fd + 6]
            for i in range(29):
                out["q"][row, i] = self.data.qpos[self.j_qpos_adr[i]]
                out["qd"][row, i] = self.data.qvel[self.j_dof_adr[i]]
            out["tau"][row] = tau
        return out

    def rollout_clips(self, clips: List[Dict], params: Dict,
                      kappa_map: Dict[str, List[int]]) -> List[Dict[str, np.ndarray]]:
        return [self.rollout_clip(c, params, kappa_map) for c in clips]
