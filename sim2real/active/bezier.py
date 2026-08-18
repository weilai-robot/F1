"""Bézier command reparameterization (paper A.3).

Command trajectories are represented by a Bézier curve per optimized command
dimension; CMA-ES optimizes only the control points, compressing the search
space vs. per-timestep commands.
"""

from __future__ import annotations

import numpy as np


def bezier_matrix(n_points: int, n_samples: int) -> np.ndarray:
    """De Casteljau basis matrix B (n_samples, n_points): curve = B @ ctrl."""
    t = np.linspace(0.0, 1.0, n_samples)[:, None]
    n = n_points - 1
    from math import comb
    B = np.zeros((n_samples, n_points))
    for i in range(n_points):
        B[:, i] = comb(n, i) * (1 - t[:, 0]) ** (n - i) * t[:, 0] ** i
    return B


def sample_curve(ctrl: np.ndarray, n_samples: int) -> np.ndarray:
    """ctrl (n_points,) -> curve (n_samples,) via Bézier basis."""
    B = bezier_matrix(ctrl.shape[0], n_samples)
    return B @ ctrl


def control_points_from_ranges(ranges: np.ndarray, n_points: int,
                               rng: np.random.Generator) -> np.ndarray:
    """Random control points inside per-dim command ranges.

    ranges: (n_dim, 2) lo/hi. Returns (n_dim, n_points) normalized to [0, 1]
    plus the ranges, so denormalize() maps back.
    """
    u = rng.uniform(0.0, 1.0, size=(ranges.shape[0], n_points))
    return u


def denormalize(u: np.ndarray, ranges: np.ndarray) -> np.ndarray:
    """Map unit-square control points back to command ranges."""
    lo = ranges[:, 0][:, None]
    hi = ranges[:, 1][:, None]
    return lo + np.clip(u, 0.0, 1.0) * (hi - lo)
