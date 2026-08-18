#!/usr/bin/env python3
"""Single-parameter landscape diagnostics (paper repo: scripts/mass_landscape.py).

Sweeps one parameter (default: base mass) over a grid, evaluates the SPI
prediction cost, and saves curve + plot under logs/mass_landscape/.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim2real"))

from spi.cost import CostWeights, PredictionCost  # noqa: E402
from spi.dataset import JIDX, LEG_JOINTS, load_clips  # noqa: E402
from spi.optimizer import build_space  # noqa: E402
from spi.rollout import MuJoCoRollouter  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--param", default="mass", choices=["mass", "com_x", "com_y", "com_z"])
    ap.add_argument("--values", type=float, nargs="+", default=None)
    ap.add_argument("--max-clips", type=int, default=20)
    ap.add_argument("--out-dir", default="logs/mass_landscape")
    args = ap.parse_args()

    import yaml
    cfg = yaml.safe_load(Path(args.config).read_text())
    clips, _ = load_clips(args.dataset)
    clips = clips[:args.max_clips]

    mjcf = (ROOT / cfg["model"]["mjcf"]).resolve()
    rollouter = MuJoCoRollouter(mjcf, base_body=cfg["model"]["base_body"],
                                foot_bodies=tuple(cfg["model"]["foot_bodies"]))
    kmap = {g["name"]: [JIDX[j] for j in g["joints"]] for g in cfg["motor_groups"]}
    weights = CostWeights.from_dict(cfg["cost"])
    mask = np.zeros(29, dtype=bool)
    for j in LEG_JOINTS:
        mask[JIDX[j]] = True
    cost_fn = PredictionCost(weights=weights, joint_mask=mask)
    body_key = cfg["model"].get("base_body_key", "base")

    space = build_space(cfg)
    nom = space.nominal_params()
    base_body = nom["bodies"][body_key]

    lo, hi = (cfg["bodies"][0]["mass_range"] if args.param == "mass"
              else [-0.15, 0.15])
    values = args.values or np.linspace(lo, hi, 15).tolist()

    costs = []
    for v in values:
        p = {"mass": base_body["mass"], "com": base_body["com"].copy(),
             "inertia": base_body["inertia"].copy()}
        if args.param == "mass":
            p["mass"] = v
        else:
            axis = {"com_x": 0, "com_y": 1, "com_z": 2}[args.param]
            p["com"] = np.array([0.0, 0.0, 0.0]); p["com"][axis] = v
        params = {"bodies": {"base": p}, "motors": nom["motors"],
                  "kappa_s": nom["kappa_s"]}
        sims = rollouter.rollout_clips(clips, params, kmap)
        c = sum(cost_fn.evaluate(sim, {"quat": cl["ref_quat"], "gyro": cl["ref_gyro"],
                                       "q": cl["ref_q"], "qd": cl["ref_qd"],
                                       "tau": cl["ref_tau"]})
                for cl, sim in zip(clips, sims))
        costs.append(c)
        print(f"[landscape] {args.param}={v:.4f} -> cost {c:.4f}")

    out = Path(args.out_dir); out.mkdir(parents=True, exist_ok=True)
    (out / "landscape.json").write_text(json.dumps(
        {"param": args.param, "values": values, "costs": costs}, indent=2))
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        plt.figure(figsize=(6, 4))
        plt.plot(values, costs, "o-")
        plt.xlabel(args.param); plt.ylabel("SPI prediction cost")
        plt.title(f"{args.param} landscape ({len(clips)} clips)")
        plt.grid(alpha=0.3)
        plt.savefig(out / "landscape.png", dpi=120, bbox_inches="tight")
        print(f"[landscape] plot -> {out/'landscape.png'}")
    except ImportError:
        pass
    best = values[int(np.argmin(costs))]
    print(f"[landscape] best {args.param} = {best:.4f} (cost {min(costs):.4f})")


if __name__ == "__main__":
    main()
