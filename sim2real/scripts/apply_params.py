#!/usr/bin/env python3
"""Apply identified SPI parameters: patch URDF/MJCF + emit DR config for
downstream remote retraining (agibot_x1_train framework).

Inputs : identified_params.json (from run_spi.py)
Outputs (under --out-dir, default sim2real/export/):
  * x1_identified.urdf          pelvis inertial patched
  * xyber_x1_identified.xml     MJCF (robot include) patched
  * dr_x1_spi.json              DR ranges re-centred on identified values
  * report.md                   human-readable diff vs nominal
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]

# agibot_x1_train / legged_gym DR knobs (x1_dh_stand_config.py domain_rand)
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


def patch_urdf(urdf_text: str, body: str, mass: float, com, inertia) -> str:
    """Replace the <inertial> of `body` link. X1 URDF link name for pelvis is
    checked by regex; falls back to first <link> block named link_base/base."""
    full = (f"{inertia[0][0]:.8f} {inertia[0][1]:.8f} {inertia[0][2]:.8f} "
            f"{inertia[1][0]:.8f} {inertia[1][1]:.8f} {inertia[1][2]:.8f} "
            f"{inertia[2][0]:.8f} {inertia[2][1]:.8f} {inertia[2][2]:.8f}")
    new_inertial = (f'<inertial>\n    <origin xyz="{com[0]:.8f} {com[1]:.8f} {com[2]:.8f}"'
                    f' rpy="0 0 0"/>\n    <mass value="{mass:.8f}"/>\n'
                    f'    <inertia ixx="{inertia[0][0]:.8f}" ixy="{inertia[0][1]:.8f}"'
                    f' ixz="{inertia[0][2]:.8f}" iyy="{inertia[1][1]:.8f}"'
                    f' iyz="{inertia[1][2]:.8f}" izz="{inertia[2][2]:.8f}"/>\n  </inertial>')
    # find link block
    pat = re.compile(r'(<link\s+name="' + re.escape(body) + r'"\s*>\s*)(.*?)(</link>)',
                     re.S)
    m = pat.search(urdf_text)
    if not m:
        # try base link names commonly used
        for alt in ("base", "link_base", "pelvis"):
            pat = re.compile(r'(<link\s+name="' + alt + r'"\s*>\s*)(.*?)(</link>)', re.S)
            m = pat.search(urdf_text)
            if m:
                break
    if not m:
        raise SystemExit("cannot locate pelvis <link> in URDF")
    block = m.group(2)
    block_new = re.sub(r"<inertial>.*?</inertial>", new_inertial, block, count=1,
                       flags=re.S)
    del full
    return urdf_text[:m.start(2)] + block_new + urdf_text[m.end(2):]


def patch_mjcf(mjcf_text: str, body: str, mass: float, com, inertia) -> str:
    """Replace <inertial .../> inside <body name="body">."""
    I = np.asarray(inertia, dtype=float)
    lam, V = np.linalg.eigh(0.5 * (I + I.T))
    if np.linalg.det(V) < 0:
        V[:, 0] = -V[:, 0]
    q = _mat2quat(V)
    full = (f'<inertial pos="{com[0]:.8f} {com[1]:.8f} {com[2]:.8f}" '
            f'mass="{mass:.8f}" '
            f'diaginertia="{lam[0]:.8f} {lam[1]:.8f} {lam[2]:.8f}" '
            f'quat="{q[0]:.8f} {q[1]:.8f} {q[2]:.8f} {q[3]:.8f}"/>')
    pat = re.compile(r'(<body\s+name="' + re.escape(body) + r'".*?>)(.*?)(</body>)', re.S)
    m = pat.search(mjcf_text)
    if not m:
        raise SystemExit(f"cannot locate <body name={body}> in MJCF")
    block_new = re.sub(r"<inertial[^>]*/>", full, m.group(2), count=1)
    return mjcf_text[:m.start(2)] + block_new + mjcf_text[m.end(2):]


def _mat2quat(R: np.ndarray) -> np.ndarray:
    from spi.rollout import mat2quat_wxyz
    return mat2quat_wxyz(R)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--params", required=True, help="identified_params.json")
    ap.add_argument("--urdf", default=None, help="source x1.urdf")
    ap.add_argument("--mjcf", default=None, help="source xyber_x1_serial.xml")
    ap.add_argument("--body", default="link_base")
    ap.add_argument("--out-dir", default=str(ROOT / "sim2real/export"))
    args = ap.parse_args()

    payload = json.loads(Path(args.params).read_text())
    p = payload["best_params"]["bodies"]["base"]
    mass, com, inertia = p["mass"], np.array(p["com"]), np.array(p["inertia"])
    print(f"[apply] identified pelvis: m={mass:.4f} com={com.round(5)} "
          f"I_diag={np.diag(inertia).round(5)}")

    out = Path(args.out_dir); out.mkdir(parents=True, exist_ok=True)

    urdf_path = Path(args.urdf) if args.urdf else _find_resource(
        ["motion_control/module/sim_module/model/mjcf/robot/xyber_x1/x1.urdf",
         "../agibot_x1_train/resources/robots/x1/urdf/x1.urdf",
         "../Humanoid_motion/module/sim_module/model/mjcf/robot/xyber_x1/x1.urdf"])
    if urdf_path and urdf_path.exists():
        text = patch_urdf(urdf_path.read_text(), args.body, mass, com, inertia)
        (out / "x1_identified.urdf").write_text(text)
        print(f"[apply] URDF -> {out/'x1_identified.urdf'}")
    else:
        print("[apply] URDF source not found; skipped (pass --urdf)")

    mjcf_path = Path(args.mjcf) if args.mjcf else _find_resource(
        ["motion_control/module/sim_module/model/mjcf/robot/xyber_x1/xyber_x1_serial.xml",
         "../Humanoid_motion/module/sim_module/model/mjcf/robot/xyber_x1/xyber_x1_serial.xml"])
    if mjcf_path and mjcf_path.exists():
        text = patch_mjcf(mjcf_path.read_text(), args.body, mass, com, inertia)
        (out / "xyber_x1_identified.xml").write_text(text)
        print(f"[apply] MJCF -> {out/'xyber_x1_identified.xml'}")

    dr = dict(DR_TEMPLATE)
    dr["identified_mass"] = mass
    dr["identified_com"] = com.tolist()
    dr["added_mass_range"] = [round(mass * 0.95, 4), round(mass * 1.05, 4)]
    dr["com_range"] = [[round(com[i] - 0.03, 5), round(com[i] + 0.03, 5)]
                       for i in range(3)]
    dr["motor_kappa"] = payload["best_params"]["motors"]
    dr["kappa_s"] = payload["best_params"]["kappa_s"]
    (out / "dr_x1_spi.json").write_text(json.dumps(dr, indent=2))
    print(f"[apply] DR  -> {out/'dr_x1_spi.json'}")

    nominal_mass = 4.3041648
    report = (f"# SPI identified parameters\n\n"
              f"* pelvis mass: {mass:.4f} kg (nominal {nominal_mass:.4f}, "
              f"Δ={mass-nominal_mass:+.4f})\n"
              f"* pelvis com: {com.round(6).tolist()}\n"
              f"* pelvis inertia diag: {np.diag(inertia).round(6).tolist()}\n"
              f"* motor kappa: {payload['best_params']['motors']}\n"
              f"* kappa_s: {payload['best_params']['kappa_s']}\n"
              f"* cost: nominal={payload.get('nominal_cost'):.4f} -> "
              f"best={payload['best_cost']:.4f}\n")
    (out / "report.md").write_text(report)
    print(f"[apply] report -> {out/'report.md'}")


def _find_resource(candidates):
    for c in candidates:
        p = (ROOT / c)
        if p.exists():
            return p
    return None


if __name__ == "__main__":
    main()
