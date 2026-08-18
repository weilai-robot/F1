"""Fisher Information via finite differences (paper Sec.4 / A.3, repo defaults).

F(θ,π) ≈ σ⁻² E[ Σ_t (∂f/∂θ)(∂f/∂θ)^T ], gradient approximated by finite
differences with perturbation ``delta_param``. Auxiliary (perturbed)
environments are re-synchronized to the main environment every
``ksync_steps`` steps so the finite difference measures local sensitivity of
the dynamics instead of the divergence of two free-running rollouts.
Objective: tr(F⁻¹) (A-optimality) + termination penalty.

numpy-only; simulator interaction is injected through the caller.
"""

from __future__ import annotations

from typing import Callable, Dict, List

import numpy as np


def fd_jacobian(perturb_step_fn: Callable[[np.ndarray, float], np.ndarray],
                theta: np.ndarray, delta: float = 0.1) -> np.ndarray:
    """Central finite-difference Jacobian.

    perturb_step_fn(theta_perturbed, delta) must return the one-step output
    difference vector (state residual vs. the synced main-env state), computed
    with aux envs resynced every ksync steps by the caller.
    Returns J with shape (state_dim, param_dim): F = sigma^-2 sum_t J^T J.
    """
    d = theta.shape[0]
    cols = []
    for i in range(d):
        tp = theta.copy(); tp[i] += delta
        tm = theta.copy(); tm[i] -= delta
        fp = perturb_step_fn(tp, +delta)
        fm = perturb_step_fn(tm, -delta)
        cols.append((fp - fm) / (2.0 * delta))
    return np.stack(cols, axis=1)


def fim_from_jacobians(jacobs: List[np.ndarray], sigma: float = 1.0) -> np.ndarray:
    """F = sigma^-2 sum_t J_t^T J_t  (param_dim x param_dim; J is
    state_dim x param_dim)."""
    F = np.zeros((jacobs[0].shape[1], jacobs[0].shape[1]))
    for J in jacobs:
        F += J.T @ J
    return F / (sigma ** 2)


def a_optimality_objective(F: np.ndarray, eps: float = 1e-9) -> float:
    """tr(F⁻¹) with Tikhonov fallback for singular F (early exploration)."""
    d = F.shape[0]
    try:
        return float(np.trace(np.linalg.inv(F + eps * np.eye(d))))
    except np.linalg.LinAlgError:
        return float(np.trace(np.linalg.pinv(F)))
