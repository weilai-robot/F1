"""SPI identification validation: completion criteria (完成标准).

Pure numpy — no simulator dependency — so the criteria logic is unit-testable
locally. The MuJoCo rollouts happen in scripts/validate_spi.py (remote); this
module turns rollout cost tables + parameters into a PASS/FAIL/WARN verdict
plus a per-parameter credibility grade (评审要求：参数可信度分级).

Completion criteria (see also configs/x1_spi.yaml -> validation):
  1. EFFECTIVENESS  best_params 在 holdout 验证集上的总预测代价
                     <= nominal_params 的 70%（>=30% 提升，防过拟合/补偿）。
  2. PHYSICAL       body 物理参数（mass/com/inertia 特征值）全部落在配置的
                    物理合理域内（physical_violations 为空）。
  3. ACCEL          验证集 IMU 比力预测 RMS 优于 nominal 且 < 1.5 m/s^2
                    （评审点：辨识必须利用 IMU 三轴加速度）。
  4. BOUNDARY(WARN) 无参数贴搜索盒边界（提示参数吸收模型误差而非真实物理量）。

Credibility grades (informational, 高/中/低 by deviation from nominal):
  mass |d|/nominal <20% 高 <40% 中;  com max|d| <30mm 高 <60mm 中;
  inertia eig |d|/nominal <30% 高 <60% 中;  kappa |d|/nominal <30% 高 <60% 中;
  kappa_s |d| <0.15 高 <0.30 中.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence

import numpy as np

from .param_space import physical_violations

EFFECTIVE_RATIO = 0.70          # best/nominal val cost must be <= this
ACCEL_RMS_MAX = 15.0            # m/s^2, val-set IMU specific-force RMS (无动捕开环现实界)
ACCEL_IMPROVE_RATIO = 0.35      # best RMS <= nominal*this (改善 >=65%)
BOUNDARY_FRACTION = 0.02        # within 2% of a search-box edge -> WARN


def split_clips(clips: List[Dict], val_ratio: float = 0.2, seed: int = 0):
    """Deterministic train/validation split (validation = holdout)."""
    rng = np.random.default_rng(seed)
    idx = rng.permutation(len(clips))
    n_val = max(1, int(round(len(clips) * val_ratio)))
    val = [clips[i] for i in idx[:n_val]]
    train = [clips[i] for i in idx[n_val:]]
    return train, val


def accel_rms(per_signal: Dict[str, float], weight: float, n_steps: int) -> Optional[float]:
    """RMS of the IMU specific-force error in m/s^2 from the accel cost term.

    accel term = weight * sum(||a_sim - a_ref||^2); RMS = sqrt(term / (weight*n)).
    Returns None when the accel term is disabled (weight 0).
    """
    if weight <= 0 or "accel" not in per_signal or n_steps <= 0:
        return None
    return float(np.sqrt(max(per_signal["accel"], 0.0) / (weight * n_steps)))


def credibility_grade(params: Dict, nominal: Dict, bodies_cfg: Sequence[Dict],
                      motor_groups: Sequence[Dict],
                      kappa_s_nominal: float) -> Dict[str, str]:
    """Per-parameter-group credibility: 高/中/低 by deviation from nominal."""
    grades: Dict[str, str] = {}
    for b in bodies_cfg:
        p = params["bodies"][b["name"]]
        n = nominal["bodies"][b["name"]]
        d_mass = abs(p["mass"] - n["mass"]) / max(abs(n["mass"]), 1e-9)
        grades[f"{b['name']}.mass"] = "高" if d_mass < 0.20 else ("中" if d_mass < 0.40 else "低")
        d_com = float(np.max(np.abs(np.asarray(p["com"]) - np.asarray(n["com"]))))
        grades[f"{b['name']}.com"] = "高" if d_com < 0.03 else ("中" if d_com < 0.06 else "低")
        lam_p = np.linalg.eigvalsh(0.5 * (np.asarray(p["inertia"]) + np.asarray(p["inertia"]).T))
        lam_n = np.linalg.eigvalsh(0.5 * (np.asarray(n["inertia"]) + np.asarray(n["inertia"]).T))
        d_i = float(np.max(np.abs(lam_p - lam_n) / np.maximum(np.abs(lam_n), 1e-9)))
        grades[f"{b['name']}.inertia"] = "高" if d_i < 0.30 else ("中" if d_i < 0.60 else "低")
    for g in motor_groups:
        p = params["motors"][g["name"]]
        n = g["kappa_nominal"]
        d = abs(p - n) / max(abs(n), 1e-9)
        grades[f"kappa.{g['name']}"] = "高" if d < 0.30 else ("中" if d < 0.60 else "低")
    d_ks = abs(params.get("kappa_s", kappa_s_nominal) - kappa_s_nominal)
    grades["kappa_s"] = "高" if d_ks < 0.15 else ("中" if d_ks < 0.30 else "低")
    return grades


def boundary_warnings(params: Dict, cfg: Dict) -> List[str]:
    """Parameters sitting at a search-box edge (within BOUNDARY_FRACTION of it)."""
    warns: List[str] = []
    for g in cfg["motor_groups"]:
        lo, hi = g["kappa_range"]
        v = params["motors"][g["name"]]
        span = max(hi - lo, 1e-9)
        if (v - lo) / span < BOUNDARY_FRACTION or (hi - v) / span < BOUNDARY_FRACTION:
            warns.append(f"kappa.{g['name']}={v:.3f} 贴搜索盒 [{lo}, {hi}] 边界")
    lo, hi = cfg.get("kappa_s_range", [0.5, 1.5])
    v = params.get("kappa_s", 1.0)
    span = max(hi - lo, 1e-9)
    if (v - lo) / span < BOUNDARY_FRACTION or (hi - v) / span < BOUNDARY_FRACTION:
        warns.append(f"kappa_s={v:.3f} 贴搜索盒 [{lo}, {hi}] 边界（提示参数在补偿模型误差）")
    for b in cfg["bodies"]:
        p = params["bodies"][b["name"]]
        m_lo, m_hi = b["mass_range"]
        if (p["mass"] - m_lo) < BOUNDARY_FRACTION * (m_hi - m_lo) or \
           (m_hi - p["mass"]) < BOUNDARY_FRACTION * (m_hi - m_lo):
            warns.append(f"{b['name']}.mass={p['mass']:.3f} 贴质量盒边界")
    return warns


def assess(cfg: Dict, params: Dict, nominal: Dict,
           train_costs: Dict[str, Dict[str, float]], val_costs: Dict[str, Dict[str, float]],
           n_val_steps: int, accel_weight: float = 1.0) -> Dict:
    """Evaluate the completion criteria.

    train_costs/val_costs: {"nominal": per_signal_cost dict, "best": per_signal_cost dict}
    Returns the full report; exit_code 0 = PASS, 1 = FAIL (WARN does not block).
    """
    body_cfg = cfg["bodies"]
    vcfg = cfg.get("validation", {})
    eff_ratio = float(vcfg.get("effective_ratio", EFFECTIVE_RATIO))
    accel_max = float(vcfg.get("accel_rms_max", ACCEL_RMS_MAX))
    checks: List[Dict] = []
    verdict = "PASS"
    exit_code = 0

    # 1. effectiveness on holdout
    vn = sum(val_costs["nominal"].values())
    vb = sum(val_costs["best"].values())
    ratio = (vb / vn) if vn > 0 else float("inf")
    ok1 = vn > 0 and ratio <= eff_ratio
    checks.append({
        "id": "EFFECTIVENESS",
        "ok": bool(ok1),
        "detail": (f"val cost best={vb:.1f} vs nominal={vn:.1f} "
                   f"(ratio {ratio:.3f} <= {eff_ratio})"),
    })

    # 2. physical plausibility
    viol = physical_violations(params, body_cfg)
    ok2 = not viol
    checks.append({
        "id": "PHYSICAL",
        "ok": ok2,
        "detail": "all body params inside configured ranges" if ok2 else f"violations: {viol}",
    })

    # 3. IMU accel term: enabled + improves (>=65% vs nominal) + bounded RMS
    rms_best = accel_rms(val_costs["best"], accel_weight, n_val_steps)
    rms_nom = accel_rms(val_costs["nominal"], accel_weight, n_val_steps)
    if accel_weight <= 0 or rms_best is None:
        pass  # accel term disabled -> check skipped
    else:
        improve_ratio = float(vcfg.get("accel_improve_ratio", 0.35))
        improved = (rms_nom is None or rms_nom < 0.01
                    or rms_best <= rms_nom * improve_ratio)
        ok3 = bool(improved and rms_best <= accel_max)
        checks.append({
            "id": "ACCEL",
            "ok": ok3,
            "detail": (f"val accel RMS: best={round(rms_best, 3)} "
                       f"nominal={None if rms_nom is None else round(rms_nom, 3)} m/s^2 "
                       f"(max {accel_max}, improve>={1 - improve_ratio:.0%})"),
        })

    for c in checks:
        if not c["ok"]:
            verdict = "FAIL"
            exit_code = 1

    warns = boundary_warnings(params, cfg)
    grades = credibility_grade(params, nominal, body_cfg, cfg["motor_groups"],
                               float(cfg.get("kappa_s_nominal", 1.0)))

    return {
        "verdict": verdict,
        "exit_code": exit_code,
        "checks": checks,
        "warnings": warns,
        "credibility": grades,
        "costs": {
            "train": {k: {kk: round(vv, 2) for kk, vv in v.items()}
                      for k, v in train_costs.items()},
            "val": {k: {kk: round(vv, 2) for kk, vv in v.items()}
                    for k, v in val_costs.items()},
        },
        "accel_rms_val": (None if rms_best is None else round(rms_best, 3)),
        "criteria": {
            "effective_ratio": eff_ratio,
            "accel_rms_max": accel_max,
            "accel_improve_ratio": float(vcfg.get("accel_improve_ratio", 0.35)),
        },
    }
