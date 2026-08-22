"""Optuna CMA-ES driver for SPI parameter identification (paper Sec.3).

Each trial: sample candidate (log-Cholesky phi per body + kappa groups + kappa_s)
-> open-loop MuJoCo replay of every clip -> multi-step prediction cost +
regularization. CMA-ES (paper: Optuna default CMA-ES, Gaussian sampler,
5 iterations) updates the search distribution.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Callable, Dict, List, Optional

import numpy as np

from .cost import CostWeights
from .param_space import (BodyParams, MotorGroup, ParamSpace,
                          physical_range_penalty, phi_to_physical)


def build_space(cfg: Dict) -> ParamSpace:
    """Build ParamSpace from the parsed x1_spi.yaml dict."""
    bodies = []
    for b in cfg["bodies"]:
        nom = b["nominal"]  # {"mass", "com", "inertia_diag" or "inertia"}
        if "inertia" in nom:
            I = np.array(nom["inertia"], dtype=float)
        else:
            I = np.diag(np.array(nom["inertia_diag"], dtype=float))
        bodies.append(BodyParams(
            body_name=b["name"],
            nominal={"mass": float(nom["mass"]),
                     "com": np.array(nom["com"], dtype=float),
                     "inertia": I},
            mass_range=tuple(b["mass_range"]),
            com_range=tuple(b["com_range"]),
            inertia_diag_range=tuple(b["inertia_diag_range"]),
        ))
    groups = [MotorGroup(g["name"], g["joints"], float(g["kappa_nominal"]),
                         tuple(g["kappa_range"])) for g in cfg["motor_groups"]]
    return ParamSpace(bodies=bodies, motor_groups=groups,
                      kappa_s_nominal=float(cfg.get("kappa_s_nominal", 1.0)),
                      kappa_s_range=tuple(cfg.get("kappa_s_range", [0.5, 1.5])))


def run_spi(clips: List[Dict], cfg: Dict, evaluate: Callable[[Dict], float],
            n_trials: int = 60, seed: int = 0,
            study_path: Optional[Path] = None,
            reg_scale: float = 0.1,
            penalty_scale: Optional[float] = None) -> Dict:
    """Run CMA-ES identification.

    evaluate(params) -> prediction cost over clips (regularization added here:
    paper Tab.3 reg terms with the 0.1 global scale). The physically-plausible
    range penalty (config optimizer.penalty_scale, default 1e5) is added to the
    objective so the identified body params stay inside the configured
    mass/com/inertia boxes.
    Returns {"best_params", "best_cost", "history"}.
    """
    import optuna

    optuna.logging.set_verbosity(optuna.logging.WARNING)
    space = build_space(cfg)
    bodies_cfg = cfg["bodies"]
    pen_scale = (penalty_scale if penalty_scale is not None
                 else float(cfg["optimizer"].get("penalty_scale", 1e5)))

    sampler = optuna.samplers.CmaEsSampler(seed=seed)
    study = optuna.create_study(direction="minimize", sampler=sampler)

    def objective(trial):
        params = space.sample(trial)
        return (evaluate(params)
                + reg_scale * space.regularization(params)
                + physical_range_penalty(params, bodies_cfg, pen_scale))

    study.optimize(objective, n_trials=n_trials)

    best = space.nominal_params()
    # reconstruct best params from the best trial's distributions
    t = study.best_trial
    for b in space.bodies:
        phi = np.array([t.params[f"{b.body_name}.{k}"] for k in
                        ["alpha", "d1", "d2", "d3", "s12", "s13", "s23", "t1", "t2", "t3"]])
        best["bodies"][b.body_name] = phi_to_physical(phi)
    for g in space.motor_groups:
        best["motors"][g.name] = t.params[f"kappa.{g.name}"]
    best["kappa_s"] = t.params["kappa_s"]

    result = {
        "best_params": best,
        "best_cost": float(study.best_value),
        "history": [(-s.value if False else s.value) for s in study.trials],
        "n_clips": len(clips),
    }
    if study_path is not None:
        study_path = Path(study_path)
        study_path.parent.mkdir(parents=True, exist_ok=True)
        study_path.write_text(json.dumps({
            "best_params": json.loads(space.to_json(best)),
            "best_cost": result["best_cost"],
            "history": result["history"],
        }, indent=2))
    return result
