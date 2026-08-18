"""SPI-Active stage-2 command-sequence optimization (paper Sec.4 / A.3).

Optimizes the input command sequence of a pre-trained multi-behavior policy
so the collected trajectory maximizes Fisher Information:
    c*_1:T = argmin_c tr(F(θ̂_1, π_c)⁻¹)   (solved with the same CMA-ES)
Commands are reparameterized by Bézier control points per dimension
(default: vx, vy, wz for the X1 cmd_vel policy), segmented into horizons of
``horizon_length_s`` seconds.

The policy is injected via ``PolicyFn``: obs -> action. For X1 today this is
the deployed rl_walk_leg ONNX (3-dim cmd_vel); a WTW-style multi-behavior
policy widens the excitation space once available (see doc/sim2real_spi.md).
"""

from __future__ import annotations

from typing import Callable, Dict, List, Optional

import numpy as np

from .bezier import bezier_matrix, denormalize


def command_objective(ctrl_points: np.ndarray, ranges: np.ndarray,
                      n_samples: int, rollout_with_commands: Callable[[np.ndarray], Dict],
                      theta: np.ndarray, fim_builder: Callable[[np.ndarray, np.ndarray], np.ndarray],
                      termination_penalty: float = 1.0e4) -> float:
    """tr(F⁻¹) + termination penalty for one candidate command sequence.

    ctrl_points: (n_dim, n_points) unit-square control points.
    rollout_with_commands(cmd_seq (n_samples, n_dim)) -> {"terminated": bool, ...}
    fim_builder(cmd_seq, theta) -> FIM built from FD rollouts at theta.
    """
    cmds = denormalize(ctrl_points, ranges)
    B = bezier_matrix(ctrl_points.shape[1], n_samples)
    cmd_seq = cmds @ B.T                      # (n_samples, n_dim)
    out = rollout_with_commands(cmd_seq)
    if out.get("terminated", False):
        return termination_penalty
    F = fim_builder(cmd_seq, theta)
    from .fim import a_optimality_objective
    return a_optimality_objective(F)


def optimize_commands(ranges: np.ndarray, n_samples: int, n_points: int,
                      rollout_with_commands: Callable, fim_builder: Callable,
                      theta: np.ndarray, n_trials: int = 40, seed: int = 0,
                      termination_penalty: float = 1.0e4) -> Dict:
    """CMA-ES over Bézier control points; returns best command sequence."""
    import optuna

    optuna.logging.set_verbosity(optuna.logging.WARNING)
    n_dim = ranges.shape[0]
    sampler = optuna.samplers.CmaEsSampler(seed=seed)
    study = optuna.create_study(direction="minimize", sampler=sampler)

    def objective(trial):
        ctrl = np.array([[trial.suggest_float(f"c{d}_p{p}", 0.0, 1.0)
                          for p in range(n_points)] for d in range(n_dim)])
        return command_objective(ctrl, ranges, n_samples, rollout_with_commands,
                                 theta, fim_builder, termination_penalty)

    study.optimize(objective, n_trials=n_trials)
    best_ctrl = np.array([[study.best_trial.params[f"c{d}_p{p}"]
                           for p in range(n_points)] for d in range(n_dim)])
    best_cmds = denormalize(best_ctrl, ranges)
    B = bezier_matrix(n_points, n_samples)
    return {"cmd_seq": best_cmds @ B.T, "cost": float(study.best_value),
            "ctrl_points": best_ctrl}
