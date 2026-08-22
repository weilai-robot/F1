"""SPI parameter space: log-Cholesky mass-inertia parameterization + tanh motor model.

Implements the parameterization of SPI-Active (arXiv:2505.14266, Sec.3 & Tab.3/4):

- Mass-inertia parameters (m, r, I) of a rigid body are reparameterized through
  the log-Cholesky decomposition of the pseudo-inertia matrix
      J = [[Sigma, h], [h^T, m]] = U U^T,
      U = exp(alpha) * [[e^{d1}, s12, s13, t1],
                        [0,     e^{d2}, s23, t2],
                        [0,     0,     e^{d3}, t3],
                        [0,     0,     0,    1  ]]
  which guarantees J ≻ 0 (physical feasibility) for any real phi in R^10.
  The unconstrained coordinates
      phi = [alpha, d1, d2, d3, s12, s13, s23, t1, t2, t3]
  are the CMA-ES optimization variables.

- Actuator model (tanh torque saturation, Eq.3):
      tau_motor = kappa_s * kappa * tanh(tau_PD / kappa)
      tau_PD = Kp (q_target - q) - Kd qdot
  with joint-group-specific kappa. kappa_s is a linear output gain in [0.5, 1.5].
  (The paper lists both a "Tanh motor gain" kappa and a "Linear motor gain"
  kappa_s as identified parameters.)

Pure numpy; no simulator dependency.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Dict, List, Sequence, Tuple

import numpy as np

PHI_NAMES = ["alpha", "d1", "d2", "d3", "s12", "s13", "s23", "t1", "t2", "t3"]


# ---------------------------------------------------------------------------
# log-Cholesky <-> physical parameter conversion
# ---------------------------------------------------------------------------

def phi_to_U(phi: np.ndarray) -> np.ndarray:
    """Build the upper-triangular U (Eq.2) from phi in R^10."""
    phi = np.asarray(phi, dtype=float)
    if phi.shape != (10,):
        raise ValueError(f"phi must have shape (10,), got {phi.shape}")
    alpha, d1, d2, d3, s12, s13, s23, t1, t2, t3 = phi
    ea = np.exp(alpha)
    U = np.zeros((4, 4))
    U[0, 0] = ea * np.exp(d1)
    U[0, 1] = ea * s12
    U[0, 2] = ea * s13
    U[0, 3] = ea * t1
    U[1, 1] = ea * np.exp(d2)
    U[1, 2] = ea * s23
    U[1, 3] = ea * t2
    U[2, 2] = ea * np.exp(d3)
    U[2, 3] = ea * t3
    U[3, 3] = ea
    return U


def phi_to_physical(phi: np.ndarray) -> Dict[str, np.ndarray]:
    """phi (R^10) -> {mass: float, com: (3,), inertia: (3,3) full matrix}.

    m = J[3,3];  h = J[:3,3] = m*r;  Sigma = J[:3,:3];
    I = tr(Sigma) * Identity(3) - Sigma   (pseudo-inertia inverse relations).
    """
    U = phi_to_U(phi)
    J = U @ U.T
    m = float(J[3, 3])
    if m <= 0.0:
        raise ValueError("non-positive mass produced; phi must be finite")
    h = J[:3, 3]
    r = h / m
    Sigma = J[:3, :3]
    I = np.trace(Sigma) * np.eye(3) - Sigma
    return {"mass": m, "com": r, "inertia": I}


def physical_to_phi(mass: float, com: Sequence[float], inertia: Sequence[Sequence[float]]) -> np.ndarray:
    """(m, r, I_full) -> phi. Used to anchor the nominal value and search box.

    Sigma = 0.5 tr(I) I3 - I; h = m r; J = [[Sigma, h],[h^T, m]]; phi from the
    *upper-triangular* Cholesky factor U with J = U U^T (paper Eq.2 form).
    numpy returns the lower factor L (J = L L^T); the upper factor in the same
    sense is obtained by reversing both axes (P L P is upper triangular and
    (P L P)(P L P)^T = P J P = J).
    """
    I = np.asarray(inertia, dtype=float)
    if I.shape != (3, 3):
        raise ValueError("inertia must be 3x3")
    if not np.allclose(I, I.T, atol=1e-9):
        I = 0.5 * (I + I.T)  # symmetrize
    Sigma = 0.5 * np.trace(I) * np.eye(3) - I
    h = float(mass) * np.asarray(com, dtype=float)
    J = np.zeros((4, 4))
    J[:3, :3] = Sigma
    J[:3, 3] = h
    J[3, 3] = float(mass)
    P = np.eye(4)[::-1]                 # axis-reversal permutation
    try:
        L = np.linalg.cholesky(P @ J @ P)   # factor of the axis-reversed J
    except np.linalg.LinAlgError as e:  # pragma: no cover - physically invalid input
        raise ValueError(f"pseudo-inertia not PD for given (m, r, I): {e}")
    U = P @ L @ P                       # upper triangular, J = U U^T
    alpha = np.log(U[3, 3])
    ea = np.exp(alpha)
    d = [np.log(U[i, i]) - alpha for i in range(3)]
    s12, s13, s23 = U[0, 1] / ea, U[0, 2] / ea, U[1, 2] / ea
    t1, t2, t3 = U[0, 3] / ea, U[1, 3] / ea, U[2, 3] / ea
    return np.array([alpha, d[0], d[1], d[2], s12, s13, s23, t1, t2, t3])


def phi_search_box(nominal: Dict[str, np.ndarray],
                   mass_range: Tuple[float, float],
                   com_range: Tuple[float, float],
                   inertia_diag_range: Tuple[float, float],
                   s_range: Tuple[float, float] = (-0.5, 0.5)) -> np.ndarray:
    """Anchor a phi-space box (10x2 lo/hi) from *physical* ranges (paper Tab.4).

    Exact relations used (U = exp(alpha) * [[...]]):
      alpha = 0.5 log m                      -> mass box maps exactly
      t_i   = r_i * exp(alpha)               -> com box maps exactly
      d_i   ≈ 0.5 log I_diag_i - alpha       -> approximate (s,t contribute to
                                               Sigma diag); feasibility holds
                                               regardless since any phi is valid.
      s_ij  shape cross-terms                -> symmetric box around 0.
    """
    lo = np.full(10, -np.inf)
    hi = np.full(10, np.inf)
    m_lo, m_hi = mass_range
    lo[0], hi[0] = 0.5 * np.log(m_lo), 0.5 * np.log(m_hi)
    m_nom = max(float(nominal["mass"]), 1e-9)
    a_nom = 0.5 * np.log(m_nom)
    i_lo, i_hi = inertia_diag_range
    # Sigma_ii >= e^{2(alpha+d_i)} and I_ii = tr(Sigma)-Sigma_ii, so the exact
    # map is loose; we anchor the d-box on the nominal mass scale. Any phi
    # sampled from this box remains physically feasible by construction.
    d_lo = 0.5 * np.log(i_lo) - a_nom
    d_hi = 0.5 * np.log(i_hi) - a_nom
    lo[1:4], hi[1:4] = d_lo, d_hi
    c_lo, c_hi = com_range
    lo[7:10], hi[7:10] = c_lo * np.exp(a_nom), c_hi * np.exp(a_nom)
    lo[4:7], hi[4:7] = s_range
    return np.stack([lo, hi], axis=1)


# ---------------------------------------------------------------------------
# tanh actuator model
# ---------------------------------------------------------------------------

def tanh_motor_torque(tau_pd: np.ndarray, kappa: float, kappa_s: float = 1.0) -> np.ndarray:
    """tau_motor = kappa_s * kappa * tanh(tau_PD / kappa)   (paper Eq.3 + kappa_s)."""
    return kappa_s * kappa * np.tanh(tau_pd / max(kappa, 1e-6))


@dataclass
class MotorGroup:
    """One identified kappa shared by a group of joints (paper: Hip/Thigh/Calf)."""
    name: str
    joints: List[str]
    kappa_nominal: float
    kappa_range: Tuple[float, float]


@dataclass
class BodyParams:
    """Identified parameters for one rigid body."""
    body_name: str
    nominal: Dict[str, np.ndarray]           # {"mass", "com", "inertia"} from MJCF/URDF
    mass_range: Tuple[float, float]
    com_range: Tuple[float, float]
    inertia_diag_range: Tuple[float, float]
    # filled by nominal_phi()
    phi_nominal: np.ndarray = field(default=None, repr=False)

    def __post_init__(self):
        if self.phi_nominal is None:
            self.phi_nominal = physical_to_phi(
                self.nominal["mass"], self.nominal["com"], self.nominal["inertia"])

    def physical_from_phi(self, phi: np.ndarray) -> Dict[str, np.ndarray]:
        return phi_to_physical(phi)


@dataclass
class ParamSpace:
    """Full SPI parameter space: bodies (log-Cholesky) + motor groups + kappa_s."""

    bodies: List[BodyParams]
    motor_groups: List[MotorGroup]
    kappa_s_nominal: float = 1.0
    kappa_s_range: Tuple[float, float] = (0.5, 1.5)

    @property
    def dim(self) -> int:
        return 10 * len(self.bodies) + len(self.motor_groups) + 1

    def sample(self, trial) -> Dict:
        """Draw one candidate from an optuna trial (uniform box, paper Tab.4)."""
        out: Dict = {"bodies": {}, "motors": {}, "kappa_s": None}
        for body in self.bodies:
            box = phi_search_box(
                {"mass": body.nominal["mass"]},
                body.mass_range, body.com_range, body.inertia_diag_range)
            phi = np.array([
                trial.suggest_float(f"{body.body_name}.{n}", box[i, 0], box[i, 1])
                for i, n in enumerate(PHI_NAMES)])
            out["bodies"][body.body_name] = body.physical_from_phi(phi)
        for g in self.motor_groups:
            out["motors"][g.name] = trial.suggest_float(
                f"kappa.{g.name}", g.kappa_range[0], g.kappa_range[1])
        out["kappa_s"] = trial.suggest_float(
            "kappa_s", self.kappa_s_range[0], self.kappa_s_range[1])
        return out

    def nominal_params(self) -> Dict:
        out: Dict = {"bodies": {}, "motors": {}, "kappa_s": self.kappa_s_nominal}
        for body in self.bodies:
            out["bodies"][body.body_name] = {
                k: (np.array(v, dtype=float) if isinstance(v, np.ndarray)
                    else np.asarray(v, dtype=float))
                for k, v in body.nominal.items()}
        for g in self.motor_groups:
            out["motors"][g.name] = g.kappa_nominal
        return out

    # -- regularization helper (paper Tab.3, scaled by 0.1) -------------------
    def regularization(self, params: Dict) -> float:
        reg = 0.0
        for body in self.bodies:
            p = params["bodies"][body.body_name]
            n = body.nominal
            reg += 0.01 * (p["mass"] - n["mass"]) ** 2
            reg += 10.0 * float(np.sum((p["com"] - n["com"]) ** 2))
            reg += 1.0 * float(np.sum((p["inertia"] - n["inertia"]) ** 2))
        for g in self.motor_groups:
            reg += 0.01 * (params["motors"][g.name] - g.kappa_nominal) ** 2
        reg += 0.1 * (params["kappa_s"] - self.kappa_s_nominal) ** 2
        return reg

    def to_json(self, params: Dict) -> str:
        def conv(o):
            if isinstance(o, np.ndarray):
                return o.tolist()
            if isinstance(o, dict):
                return {k: conv(v) for k, v in o.items()}
            return o
        return json.dumps(conv(params), indent=2)


# ---------------------------------------------------------------------------
# Physical plausibility enforcement
# ---------------------------------------------------------------------------

def physical_violations(params: Dict, bodies_cfg: Sequence[Dict]) -> Dict[str, Dict[str, float]]:
    """Signed violation magnitudes of body physical params vs configured ranges.

    bodies_cfg entries are the parsed yaml body dicts (name/mass_range/com_range/
    inertia_diag_range). Returns {body_name: {param: violation>0}} — empty dict
    when everything is inside the (physically plausible) ranges. The phi-space
    search box maps mass/com exactly but only anchors inertia, so inertia can
    exceed its intended range — this check closes that gap.
    """
    out: Dict[str, Dict[str, float]] = {}
    for b in bodies_cfg:
        p = params["bodies"][b["name"]]
        viol: Dict[str, float] = {}
        m_lo, m_hi = b["mass_range"]
        if p["mass"] < m_lo:
            viol["mass"] = m_lo - p["mass"]
        elif p["mass"] > m_hi:
            viol["mass"] = p["mass"] - m_hi
        c_lo, c_hi = b["com_range"]
        for i, ax in enumerate("xyz"):
            if p["com"][i] < c_lo:
                viol[f"com_{ax}"] = c_lo - p["com"][i]
            elif p["com"][i] > c_hi:
                viol[f"com_{ax}"] = p["com"][i] - c_hi
        i_lo, i_hi = b["inertia_diag_range"]
        lam = np.linalg.eigvalsh(0.5 * (np.asarray(p["inertia"]) + np.asarray(p["inertia"]).T))
        for i, ax in enumerate("xyz"):
            if lam[i] < i_lo:
                viol[f"inertia_{ax}"] = i_lo - lam[i]
            elif lam[i] > i_hi:
                viol[f"inertia_{ax}"] = lam[i] - i_hi
        if viol:
            out[b["name"]] = viol
    return out


def physical_range_penalty(params: Dict, bodies_cfg: Sequence[Dict],
                           scale: float = 1e4) -> float:
    """Hard constraint: quadratic penalty per unit of violation.

    Units: mass [kg]^2, com [m]^2, inertia [kg·m^2]^2. With the default scale
    1e4, a violation of ~1.7 kg·m^2 (the overshoot of the pre-constraint run)
    costs ~3e4 per axis — comparable to the prediction-cost scale (~1e5), so
    the optimizer is pushed back into the physically plausible box.
    """
    total = 0.0
    for body, viol in physical_violations(params, bodies_cfg).items():
        for v in viol.values():
            total += v * v
    return scale * total
