"""SPI multi-step prediction cost (paper Tab.3, adapted for X1 without mocap).

J(theta, {c_k}) = sum_k sum_t  [ w_quat  * (1 - <q, q_r>^2)
                               + w_w     * ||w - w_r||^2
                               + w_a     * ||a_imu - a_imu,r||^2   (body frame)
                               + w_q     * ||q_j - q_j,r||^2   (masked joints)
                               + w_qd    * ||qd - qd_r||^2     (masked joints)
                               + w_tau   * ||tau - tau_r||^2   (finite refs)
                               + reg_scale * regularization(theta) ]

Paper Tab.3 coefficients with global scaling (vel x0.5, torque x0.2, reg x0.1).
X1 has no motion capture -> base position / linear-velocity weights default 0;
instead the logged IMU 3-axis specific force (imu_accel_*) is compared against
the simulated Rᵀ(a−g) to constrain the base translational dynamics (mass/CoM).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np


@dataclass
class CostWeights:
    # base prediction (paper Tab.3, after global scaling where applicable)
    base_pos: float = 0.0      # 4.0 in paper; 0 on X1 (no mocap)
    base_linvel: float = 0.0   # 2.0*0.5; 0 on X1
    base_quat: float = 2.0
    base_angvel: float = 0.5 * 0.5
    # IMU 3-axis specific force (m/s^2, body frame). No paper counterpart:
    # replaces the missing base linear dynamics term, constrains mass/CoM.
    base_accel: float = 1.0
    # box-filter window (samples @100Hz, 10 = 0.1 s) applied to BOTH sim and
    # ref specific force before scoring: removes contact-impact spikes that
    # dominate the raw signal in open-loop replays and carry no identifiability
    # information (低频动力学才包含质量/质心信息)
    accel_filter_win: int = 10
    # joint prediction
    q: float = 3.0
    qd: float = 0.1
    tau: float = 0.01 * 0.2
    # regularization scale applied to ParamSpace.regularization()
    reg_scale: float = 0.1

    @staticmethod
    def from_dict(d: Optional[Dict[str, float]]) -> "CostWeights":
        w = CostWeights()
        if d:
            for k, v in d.items():
                if not hasattr(w, k):
                    raise KeyError(f"unknown cost weight '{k}'")
                if isinstance(getattr(w, k), int):
                    setattr(w, k, int(v))
                else:
                    setattr(w, k, float(v))
        return w


def quat_err(q: np.ndarray, q_ref: np.ndarray) -> np.ndarray:
    """1 - <q, q_ref>^2 per step; inputs (..., 4) wxyz, sign-invariant."""
    q = q / np.maximum(np.linalg.norm(q, axis=-1, keepdims=True), 1e-12)
    q_ref = q_ref / np.maximum(np.linalg.norm(q_ref, axis=-1, keepdims=True), 1e-12)
    dot = np.sum(q * q_ref, axis=-1)
    return 1.0 - dot ** 2


def box_filter(a: np.ndarray, win: int) -> np.ndarray:
    """Moving-average (box) filter per column; win=1 -> identity.

    Edges are extended by replication (np.pad mode='edge') so constant
    signals stay constant and clip boundaries are not distorted. The output
    has the same length as the input.
    """
    win = int(win)
    if win <= 1:
        return a
    a = np.asarray(a, dtype=float)
    n = a.shape[0]
    pad = win // 2
    k = np.ones(win) / win
    out = np.empty_like(a)
    for i in range(a.shape[1]):
        p = np.pad(a[:, i], (pad, pad), mode="edge")
        y = np.convolve(p, k, mode="same")
        out[:, i] = y[pad:pad + n]
    return out


def clip_cost(cost: np.ndarray) -> float:
    """Robust aggregation: clip extreme squared errors so a single diverged
    clip does not dominate the CMA-ES ranking (paper normalizes coefficients
    on a reference dataset; clipping plays the same robustifying role)."""
    return float(np.sum(np.clip(cost, None, 1e6)))


@dataclass
class PredictionCost:
    weights: CostWeights = field(default_factory=CostWeights)
    joint_mask: Optional[np.ndarray] = None   # bool (29,) - logged leg joints

    def evaluate(self, sim: Dict[str, np.ndarray], ref: Dict[str, np.ndarray]) -> float:
        """sim/ref: dicts with quat (n,4), gyro (n,3), accel (n,3), q (n,29),
        qd (n,29), tau (n,29). NaN reference entries are excluded per-term."""
        w = self.weights
        c = 0.0
        n = sim["quat"].shape[0]
        if w.base_quat > 0:
            c += w.base_quat * np.sum(quat_err(sim["quat"], ref["quat"]))
        if w.base_angvel > 0 and ref.get("gyro") is not None:
            d2 = np.sum((sim["gyro"] - ref["gyro"]) ** 2, axis=1)
            c += w.base_angvel * np.sum(np.nan_to_num(d2))
        if w.base_accel > 0 and ref.get("accel") is not None:
            sim_a = box_filter(sim["accel"], w.accel_filter_win)
            ref_a = box_filter(ref["accel"], w.accel_filter_win)
            d2 = np.sum((sim_a - ref_a) ** 2, axis=1)
            ok = np.isfinite(ref_a).all(axis=1) & np.isfinite(sim_a).all(axis=1)
            c += w.base_accel * float(np.sum(d2[ok]))
        mask = self.joint_mask
        if mask is None:
            mask = np.ones(sim["q"].shape[1], dtype=bool)
        if w.q > 0:
            d2 = np.sum((sim["q"][:, mask] - ref["q"][:, mask]) ** 2, axis=1)
            d2 = np.where(np.isfinite(ref["q"][:, mask]).all(axis=1), d2, 0.0)
            c += w.q * np.sum(d2)
        if w.qd > 0:
            d2 = np.sum((sim["qd"][:, mask] - ref["qd"][:, mask]) ** 2, axis=1)
            d2 = np.where(np.isfinite(ref["qd"][:, mask]).all(axis=1), d2, 0.0)
            c += w.qd * np.sum(d2)
        if w.tau > 0 and ref.get("tau") is not None:
            err2 = (sim["tau"] - ref["tau"]) ** 2
            ok = np.isfinite(ref["tau"]) & np.isfinite(sim["tau"])
            c += w.tau * float(np.sum(err2[ok]))
        del n
        return clip_cost(np.asarray(c))


def total_cost(cost_fn: PredictionCost, clips: List[Dict], sims: List[Dict],
               params: Dict, space) -> float:
    """Prediction cost over all clips + parameter regularization."""
    pred = sum(cost_fn.evaluate(sim, clip_refs(clip)) for clip, sim in zip(clips, sims))
    reg = cost_fn.weights.reg_scale * space.regularization(params)
    return pred + reg


def clip_refs(clip: Dict) -> Dict[str, np.ndarray]:
    return {"quat": clip["ref_quat"], "gyro": clip["ref_gyro"],
            "accel": clip.get("ref_accel"), "q": clip["ref_q"],
            "qd": clip["ref_qd"], "tau": clip["ref_tau"]}


def per_signal_cost(cost_fn: PredictionCost, sim: Dict[str, np.ndarray],
                    ref: Dict[str, np.ndarray]) -> Dict[str, float]:
    """Break the prediction cost down per signal (for validation reports)."""
    w = cost_fn.weights
    out: Dict[str, float] = {}
    n = sim["quat"].shape[0]
    if w.base_quat > 0:
        out["quat"] = float(w.base_quat * np.sum(quat_err(sim["quat"], ref["quat"])))
    if w.base_angvel > 0 and ref.get("gyro") is not None:
        d2 = np.sum((sim["gyro"] - ref["gyro"]) ** 2, axis=1)
        out["angvel"] = float(w.base_angvel * np.sum(np.nan_to_num(d2)))
    if w.base_accel > 0 and ref.get("accel") is not None:
        sim_a = box_filter(sim["accel"], w.accel_filter_win)
        ref_a = box_filter(ref["accel"], w.accel_filter_win)
        d2 = np.sum((sim_a - ref_a) ** 2, axis=1)
        ok = np.isfinite(ref_a).all(axis=1) & np.isfinite(sim_a).all(axis=1)
        out["accel"] = float(w.base_accel * float(np.sum(d2[ok])))
    mask = cost_fn.joint_mask
    if mask is None:
        mask = np.ones(sim["q"].shape[1], dtype=bool)
    if w.q > 0:
        d2 = np.sum((sim["q"][:, mask] - ref["q"][:, mask]) ** 2, axis=1)
        ok = np.isfinite(ref["q"][:, mask]).all(axis=1)
        out["q"] = float(w.q * float(np.sum(d2[ok])))
    if w.qd > 0:
        d2 = np.sum((sim["qd"][:, mask] - ref["qd"][:, mask]) ** 2, axis=1)
        ok = np.isfinite(ref["qd"][:, mask]).all(axis=1)
        out["qd"] = float(w.qd * float(np.sum(d2[ok])))
    if w.tau > 0 and ref.get("tau") is not None:
        err2 = (sim["tau"] - ref["tau"]) ** 2
        ok = np.isfinite(ref["tau"]) & np.isfinite(sim["tau"])
        out["tau"] = float(w.tau * float(np.sum(err2[ok])))
    return out
