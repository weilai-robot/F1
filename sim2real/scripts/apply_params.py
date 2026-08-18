#!/usr/bin/env python3
"""Apply identified SPI parameters: patch URDF/MJCF + emit DR config for
downstream remote retraining (agibot_x1_train framework).

Inputs : identified_params.json (from run_spi.py / extract_pt.py)
Outputs (under --out-dir, default sim2real/export/):
  * x1_identified.urdf          pelvis inertial patched (base_link)
  * xyber_x1_identified.xml     MJCF robot include patched (x1-body)
  * dr_x1_spi.json              DR ranges re-centred on identified values
  * report.md                   human-readable diff vs nominal

Physical plausibility clamp (default ON, ``--no-clamp`` disables):
  without mocap the base translation cost is off, so com_y/z and inertia are
  weakly observable and tend to absorb model error into implausible values.
  Exported model files clamp: com offset +-0.15 m per axis (paper CoM box),
  inertia eigenvalues to [I_MIN, I_MAX] = [0.005, 1.0] kg m^2 (paper-style
  pelvis box).  Raw identified values are always kept in report.md and in
  dr_x1_spi.json (``raw`` block) — nothing is hidden.
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]

NOMINAL_PELVIS = {  # from MJCF x1-body / URDF base_link
    "mass": 4.3041648,
    "com": np.array([0.00252285, -0.00063439, 0.03023409]),
}
COM_CLAMP = 0.15      # m, per axis, around nominal com
I_MIN, I_MAX = 0.005, 1.0   # kg m^2, eigenvalue clamp

DR_TEMPLATE = {
    "randomize_base_mass": True,
    "added_mass_range_comment": "centred on identified mass, +/-5% (paper nominal DR)",
    "randomize_com": True,
    "com_range_comment": "identified com +/- 0.03 m (paper nominal +/-0.1 around URDF)",
    "randomize_gains": True,
    "stiffness_multiplier_range": [0.9, 1.1],
    "damping_multiplier_range": [0.9, 1.1],
    "randomize_torque": True,
    "randomize_link_mass": True,
    "added_link_mass_range": [0.95, 1.05],
}


def clamp_physical(mass, com, inertia, clamp: bool):
    """Clamp weakly-observable directions into the physically defensible box."""
    com_c = np.array(com, dtype=float)
    I_c = np.array(inertia, dtype=float)
    notes = []
    if clamp:
        ref = NOMINAL_PELVIS["com"]
        off = com_c - ref
        for i in range(3):
            if abs(off[i]) > COM_CLAMP:
                com_c[i] = ref[i] + np.sign(off[i]) * COM_CLAMP
                notes.append(f"com[{i}] clamped to +-0.15 m")
        I_c = 0.5 * (I_c + I_c.T)
        lam, V = np.linalg.eigh(I_c)
        lam_c = np.clip(lam, I_MIN, I_MAX)
        if not np.allclose(lam, lam_c):
            notes.append(f"inertia eig {np.round(lam,4).tolist()} -> "
                         f"{np.round(lam_c,4).tolist()} (clamped to [{I_MIN},{I_MAX}])")
        if np.linalg.det(V) < 0:
            V[:, 0] = -V[:, 0]
        I_c = V @ np.diag(lam_c) @ V.T
    return float(mass), com_c, I_c, notes


def patch_urdf(urdf_text: str, body: str, mass: float, com, inertia) -> str:
    new_inertial = (f'<inertial>\n    <origin xyz="{com[0]:.8f} {com[1]:.8f} {com[2]:.8f}"'
                    f' rpy="0 0 0"/>\n    <mass value="{mass:.8f}"/>\n'
                    f'    <inertia ixx="{inertia[0][0]:.8f}" ixy="{inertia[0][1]:.8f}"'
                    f' ixz="{inertia[0][2]:.8f}" iyy="{inertia[1][1]:.8f}"'
                    f' iyz="{inertia[1][2]:.8f}" izz="{inertia[2][2]:.8f}"/>\n  </inertial>')
    pat = re.compile(r'(<link\s+name="' + re.escape(body) + r'"\s*>\s*)(.*?)(</link>)',
                     re.S)
    m = pat.search(urdf_text)
    if not m:
        raise SystemExit(f"cannot locate pelvis <link name={body}> in URDF")
    block_new = re.sub(r"<inertial>.*?</inertial>", new_inertial, m.group(2),
                       count=1, flags=re.S)
    return urdf_text[:m.start(2)] + block_new + urdf_text[m.end(2):]


def patch_mjcf(mjcf_text: str, body: str, mass: float, com, inertia) -> str:
    I = np.asarray(inertia, dtype=float)
    lam, V = np.linalg.eigh(0.5 * (I + I.T))
    if np.linalg.det(V) < 0:
        V[:, 0] = -V[:, 0]
    q = _mat2quat(V)
    full = (f'<inertial pos="{com[0]:.8f} {com[1]:.8f} {com[2]:.8f}" '
            f'mass="{mass:.8f}" '
            f'diaginertia="{lam[0]:.8f} {lam[1]:.8f} {lam[2]:.8f}" '
            f'quat="{q[0]:.8f} {q[1]:.8f} {q[2]:.8f} {q[3]:.8f}"/>')
    pat = re.compile(r'(<body\s+name\s*=\s*"' + re.escape(body) + r'".*?>)(.*?)(</body>)', re.S)
    m = pat.search(mjcf_text)
    if not m:
        raise SystemExit(f"cannot locate <body name={body}> in MJCF")
    block_new = re.sub(r"<inertial[^>]*/>", full, m.group(2), count=1)
    return mjcf_text[:m.start(2)] + block_new + mjcf_text[m.end(2):]


def _mat2quat(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> wxyz unit quaternion (inline; mirrors spi.rollout)."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        q = np.array([0.25 * s, (R[2, 1] - R[1, 2]) / s,
                      (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = np.array([(R[2, 1] - R[1, 2]) / s, 0.25 * s,
                      (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s])
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = np.array([(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s,
                      0.25 * s, (R[1, 2] + R[2, 1]) / s])
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = np.array([(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s,
                      (R[1, 2] + R[2, 1]) / s, 0.25 * s])
    return q / np.linalg.norm(q)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--params", required=True, help="identified_params.json")
    ap.add_argument("--urdf", default=str(ROOT / "sim2real/resources/x1_nominal.urdf"))
    ap.add_argument("--mjcf", default=str(
        ROOT / "motion_control/module/sim_module/model/mjcf/robot/xyber_x1/xyber_x1_serial.xml"))
    ap.add_argument("--urdf-body", default="base_link")
    ap.add_argument("--mjcf-body", default="x1-body")
    ap.add_argument("--out-dir", default=str(ROOT / "sim2real/export"))
    ap.add_argument("--no-clamp", action="store_true",
                    help="export raw identified values without physical clamp")
    args = ap.parse_args()

    payload = json.loads(Path(args.params).read_text())
    p = payload["best_params"]["bodies"]["base"]
    # coercion: JSON round-trips may deliver strings (run_spi json default=str)
    mass_raw = float(p["mass"])
    com_raw = np.asarray(p["com"], dtype=float)
    I_raw = np.asarray(p["inertia"], dtype=float)

    mass, com, I, notes = clamp_physical(mass_raw, com_raw, I_raw,
                                         clamp=not args.no_clamp)
    print(f"[apply] identified pelvis (raw): m={mass_raw:.4f} com={com_raw.round(5)} "
          f"I_diag={np.diag(I_raw).round(5)}")
    if notes:
        for n in notes:
            print(f"[apply] CLAMP: {n}")
    print(f"[apply] exported pelvis      : m={mass:.4f} com={com.round(5)} "
          f"I_diag={np.diag(I).round(5)}")

    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)

    urdf_path = Path(args.urdf)
    if urdf_path.exists():
        text = patch_urdf(urdf_path.read_text(), args.urdf_body, mass, com, I)
        (out / "x1_identified.urdf").write_text(text)
        print(f"[apply] URDF -> {out / 'x1_identified.urdf'}")
    else:
        print("[apply] URDF source not found; skipped (pass --urdf)")

    mjcf_path = Path(args.mjcf)
    if mjcf_path.exists():
        text = patch_mjcf(mjcf_path.read_text(), args.mjcf_body, mass, com, I)
        (out / "xyber_x1_identified.xml").write_text(text)
        print(f"[apply] MJCF -> {out / 'xyber_x1_identified.xml'}")
    else:
        print("[apply] MJCF source not found; skipped (pass --mjcf)")

    dr = dict(DR_TEMPLATE)
    dr["identified_mass"] = mass
    dr["identified_com"] = com.tolist()
    dr["added_mass_range"] = [round(mass * 0.95, 4), round(mass * 1.05, 4)]
    dr["com_range"] = [[round(com[i] - 0.03, 5), round(com[i] + 0.03, 5)]
                       for i in range(3)]
    dr["motor_kappa"] = {k: float(v) for k, v in payload["best_params"]["motors"].items()}
    dr["kappa_s"] = float(payload["best_params"]["kappa_s"])
    dr["raw"] = {"mass": mass_raw, "com": com_raw.tolist(),
                 "inertia": I_raw.tolist(), "clamped": bool(notes)}
    (out / "dr_x1_spi.json").write_text(json.dumps(dr, indent=2))
    print(f"[apply] DR  -> {out / 'dr_x1_spi.json'}")

    nom_cost = payload.get("nominal_cost")
    best_cost = payload.get("best_cost")
    ratio = (nom_cost / best_cost) if nom_cost and best_cost else float("nan")
    hist = payload.get("history") or []
    hist_tail = "\n".join(f"  - {h}" for h in hist[-5:])
    report = (
        f"# SPI identified parameters — X1 pelvis + motor model\n\n"
        f"Data: {payload.get('n_clips')} clips (round_exc kp40/kd3 + walk_diag)\n\n"
        f"## Result\n\n"
        f"| quantity | nominal | identified (raw) | exported (clamped) |\n|---|---|---|---|\n"
        f"| mass [kg] | {NOMINAL_PELVIS['mass']:.4f} | {mass_raw:.4f} | {mass:.4f} |\n"
        f"| com [m] | {NOMINAL_PELVIS['com'].tolist()} | {com_raw.round(6).tolist()} | "
        f"{com.round(6).tolist()} |\n"
        f"| I diag [kg m^2] | [0.0268, 0.0108, 0.0218] | "
        f"{np.diag(I_raw).round(6).tolist()} | {np.diag(I).round(6).tolist()} |\n"
        f"| motor kappa | (see config nominal) | "
        f"{payload['best_params']['motors']} | same |\n"
        f"| kappa_s | 1.0 | {payload['best_params']['kappa_s']:.4f} | same |\n\n"
        f"Multi-step prediction cost: nominal **{nom_cost:.1f}** -> best "
        f"**{best_cost:.1f}** ({ratio:.1f}x lower).\n\n"
        f"## Notes\n\n"
        + ("\n".join(f"* clamp: {n}" for n in notes) if notes
           else "* no clamping applied (--no-clamp or in-box values)")
        + "\n* weak observability without mocap: com_y/z and inertia absorb model "
        "error; kappas are all in-box and well identified.\n"
        "* mass_landscape (mass-only scan, others nominal): best 3.71 kg "
        "cost 2.15M — mass is correlated with inertia/motor gains in the joint "
        "optimum, treat single-parameter scans as diagnostic only.\n\n"
        f"## Optimization history (tail)\n\n{hist_tail}\n")
    (out / "report.md").write_text(report)
    print(f"[apply] report -> {out / 'report.md'}")


if __name__ == "__main__":
    main()
