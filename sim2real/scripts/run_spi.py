#!/usr/bin/env python3
"""Run SPI system identification (CMA-ES over log-Cholesky + motor kappas).

Requires mujoco + optuna + pyyaml (installed by remote_sysid.sh on the
gradmotion image, or manually on any workstation).

Usage:
  python sim2real/scripts/run_spi.py \
      --config sim2real/configs/x1_spi.yaml \
      --dataset sim2real/data/x1_clips.npz \
      --out-dir logs/spi_sysid
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
from spi.dataset import FULL_JOINT_ORDER, JIDX, LEG_JOINTS, load_clips  # noqa: E402
from spi.optimizer import build_space, run_spi  # noqa: E402
from spi.rollout import MuJoCoRollouter  # noqa: E402
from spi.validate import split_clips  # noqa: E402


def kappa_map_from_cfg(cfg) -> dict:
    out = {}
    for g in cfg["motor_groups"]:
        out[g["name"]] = [JIDX[j] for j in g["joints"]]
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--out-dir", default="logs/spi_sysid")
    ap.add_argument("--n-trials", type=int, default=None)
    ap.add_argument("--max-clips", type=int, default=None)
    args = ap.parse_args()

    import yaml
    cfg = yaml.safe_load(Path(args.config).read_text())
    clips, meta = load_clips(args.dataset)
    if args.max_clips:
        clips = clips[:args.max_clips]
    val_ratio = cfg["data"].get("val_ratio", 0.2)
    train_clips, val_clips = split_clips(clips, val_ratio,
                                         seed=cfg["data"].get("seed", 0))
    print(f"[spi] {len(clips)} clips loaded (meta: {meta.get('n_clips')} total) -> "
          f"train {len(train_clips)} / val {len(val_clips)}")

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

    def evaluate(params: dict, clips_=None) -> float:
        # rollout looks up params["bodies"]["base"]; config key may differ
        clips_ = train_clips if clips_ is None else clips_
        bodies = dict(params["bodies"])
        if rollouter._base_name not in bodies:
            bodies[rollouter._base_name] = bodies[space_cfg_body_name]
        p = {"bodies": bodies, "motors": params["motors"],
             "kappa_s": params["kappa_s"]}
        sims = rollouter.rollout_clips(clips_, p, kmap)
        return sum(cost_fn.evaluate(sim, {"quat": c["ref_quat"], "gyro": c["ref_gyro"],
                                          "accel": c.get("ref_accel"),
                                          "q": c["ref_q"], "qd": c["ref_qd"],
                                          "tau": c["ref_tau"]})
                   for c, sim in zip(clips_, sims))

    # sanity: nominal-parameter cost
    space = build_space(cfg)
    nom = space.nominal_params()
    cost0 = evaluate(nom)
    print(f"[spi] nominal-params prediction cost: {cost0:.4f}")

    n_trials = args.n_trials or cfg["optimizer"]["n_trials"]
    out_dir = Path(args.out_dir)
    result = run_spi(train_clips, cfg, evaluate, n_trials=n_trials,
                     seed=cfg["optimizer"].get("seed", 0),
                     study_path=out_dir / "study.json",
                     reg_scale=weights.reg_scale)

    # holdout validation cost of the best params
    val_best = evaluate(result["best_params"], val_clips)
    val_nominal = evaluate(nom, val_clips)
    result["val_cost_best"] = float(val_best)
    result["val_cost_nominal"] = float(val_nominal)

    print(f"[spi] best cost {result['best_cost']:.4f} "
          f"(nominal was {cost0:.4f})")
    print(f"[spi] holdout val cost: best {val_best:.4f} vs nominal {val_nominal:.4f}")
    print(json.dumps(result["best_params"], indent=2, default=str))

    # artifacts under logs/<exp>/ for gradmotion SDK pickup
    gm_dir = out_dir / "gm_play"
    gm_dir.mkdir(parents=True, exist_ok=True)
    payload = {"best_params": result["best_params"], "best_cost": result["best_cost"],
               "nominal_cost": float(cost0),
               "val_cost_best": float(val_best), "val_cost_nominal": float(val_nominal),
               "history": result["history"],
               "n_clips": len(clips), "n_train": len(train_clips), "n_val": len(val_clips)}
    (gm_dir / "identified_params.json").write_text(
        json.dumps(payload, indent=2, default=str))
    try:
        import torch
        torch.save(payload, gm_dir / "identified_params.pt")
        print(f"[spi] saved {gm_dir/'identified_params.pt'}")
    except ImportError:
        print("[spi] torch not available; json artifact only")
    print("[spi] done")


if __name__ == "__main__":
    main()
