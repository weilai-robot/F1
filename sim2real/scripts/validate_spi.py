#!/usr/bin/env python3
"""SPI validation: holdout-set completion criteria (完成标准) verdict.

Requires mujoco + pyyaml (remote image), like run_spi.py. Reads the identified
params from a run, re-rollouts with *both* nominal and identified parameters on
the holdout split of the dataset, and produces:

  <out-dir>/validation.json   full report (verdict/checks/warnings/credibility)
  stdout summary + exit code: 0 = PASS, 1 = FAIL, 2 = error

Usage:
  python sim2real/scripts/validate_spi.py \
      --config sim2real/configs/x1_spi.yaml \
      --dataset sim2real/data/x1_clips.npz \
      --params logs/spi_sysid/gm_play/identified_params.json \
      --out-dir logs/spi_sysid
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim2real"))

from spi.cost import CostWeights, PredictionCost, per_signal_cost  # noqa: E402
from spi.dataset import FULL_JOINT_ORDER, JIDX, LEG_JOINTS, load_clips  # noqa: E402
from spi.optimizer import build_space  # noqa: E402
from spi.rollout import MuJoCoRollouter  # noqa: E402
from spi.validate import assess, split_clips  # noqa: E402


def _parse_nd(s: str) -> np.ndarray:
    """Parse a numpy repr string like '[-0.06 -0.02 -0.001]' or the 3x3
    '[[a b c] [d e f] [g h i]]' (space-separated, array2string style)."""
    flat = re.sub(r"[\[\]\n]", " ", s)
    return np.fromstring(flat, sep=" ")


def coerce_params(raw: Dict) -> Dict:
    """Normalize params loaded from identified_params.json.

    run_spi.py serializes with json.dumps(..., default=str), which turns
    np.ndarray values (com, inertia) into their repr *string*; coerce them
    back into arrays so MuJoCo accepts them.
    """
    out = {"bodies": {}, "motors": dict(raw.get("motors", {})),
           "kappa_s": raw.get("kappa_s", 1.0)}
    for name, b in raw["bodies"].items():
        com = b["com"]
        if isinstance(com, str):
            com = _parse_nd(com)
        inertia = b["inertia"]
        if isinstance(inertia, str):
            inertia = _parse_nd(inertia).reshape(3, 3)
        out["bodies"][name] = {"mass": float(b["mass"]),
                               "com": np.asarray(com, dtype=float),
                               "inertia": np.asarray(inertia, dtype=float)}
    return out


def kappa_map_from_cfg(cfg) -> dict:
    out = {}
    for g in cfg["motor_groups"]:
        out[g["name"]] = [JIDX[j] for j in g["joints"]]
    return out


def per_signal_sums(cost_fn, clips, sims) -> Dict[str, float]:
    total: Dict[str, float] = {}
    for clip, sim in zip(clips, sims):
        for k, v in per_signal_cost(cost_fn, sim, {
            "quat": clip["ref_quat"], "gyro": clip["ref_gyro"],
            "accel": clip.get("ref_accel"), "q": clip["ref_q"],
            "qd": clip["ref_qd"], "tau": clip["ref_tau"]}).items():
            total[k] = total.get(k, 0.0) + v
    return total


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--params", required=True)
    ap.add_argument("--out-dir", default="logs/spi_sysid")
    ap.add_argument("--val-ratio", type=float, default=None)
    args = ap.parse_args()

    import yaml
    cfg = yaml.safe_load(Path(args.config).read_text())
    clips, meta = load_clips(args.dataset)
    val_ratio = args.val_ratio if args.val_ratio is not None \
        else cfg["data"].get("val_ratio", 0.2)
    train_clips, val_clips = split_clips(clips, val_ratio, seed=cfg["data"].get("seed", 0))
    print(f"[validate] {len(clips)} clips -> train {len(train_clips)} / val {len(val_clips)}")

    with open(args.params) as f:
        payload = json.load(f)
    params = coerce_params(payload["best_params"])

    mjcf = (ROOT / cfg["model"]["mjcf"]).resolve()
    rollouter = MuJoCoRollouter(mjcf, base_body=cfg["model"]["base_body"],
                                foot_bodies=tuple(cfg["model"]["foot_bodies"]))
    kmap = kappa_map_from_cfg(cfg)
    space_cfg_body_name = cfg["bodies"][0]["name"]
    weights = CostWeights.from_dict(cfg["cost"])
    joint_mask = np.zeros(29, dtype=bool)
    for j in LEG_JOINTS:
        joint_mask[JIDX[j]] = True
    cost_fn = PredictionCost(weights=weights, joint_mask=joint_mask)
    space = build_space(cfg)
    nominal = space.nominal_params()

    def rollout_costs(clips_, p):
        bodies = dict(p["bodies"])
        if rollouter._base_name not in bodies:
            bodies[rollouter._base_name] = bodies[space_cfg_body_name]
        pp = {"bodies": bodies, "motors": p["motors"], "kappa_s": p["kappa_s"]}
        sims = rollouter.rollout_clips(clips_, pp, kmap)
        return per_signal_sums(cost_fn, clips_, sims)

    train_costs = {"nominal": rollout_costs(train_clips, nominal),
                   "best": rollout_costs(train_clips, params)}
    val_costs = {"nominal": rollout_costs(val_clips, nominal),
                 "best": rollout_costs(val_clips, params)}

    n_val_steps = sum(c["n"] for c in val_clips)
    report = assess(cfg, params, nominal, train_costs, val_costs,
                    n_val_steps, accel_weight=weights.base_accel)

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "validation.json").write_text(json.dumps(report, indent=2, ensure_ascii=False))
    print(json.dumps(report, indent=2, ensure_ascii=False))
    print(f"[validate] verdict: {report['verdict']} (exit {report['exit_code']})")
    print(f"[validate] report -> {out_dir / 'validation.json'}")
    return report["exit_code"]


if __name__ == "__main__":
    sys.exit(main())
