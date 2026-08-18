#!/usr/bin/env python3
"""Parse real-robot logger CSVs into an SPI clip dataset (.npz).

Usage (remote / any machine with numpy+pyyaml):
  python sim2real/scripts/prepare_dataset.py \
      --config sim2real/configs/x1_spi.yaml --out sim2real/data/x1_clips.npz
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim2real"))

from spi.dataset import parse_csv, save_clips, segment_clips  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--h-min", type=float, default=None)
    ap.add_argument("--h-max", type=float, default=None)
    ap.add_argument("--max-clips", type=int, default=None)
    args = ap.parse_args()

    import yaml
    cfg = yaml.safe_load(Path(args.config).read_text())

    all_clips = []
    for src in cfg["data"]["sources"]:
        kp = src.get("kp", 40.0)
        kd = src.get("kd", 3.0)
        kp = kp if isinstance(kp, dict) else {j: float(kp) for j in
                                              cfg.get("_leg_joints", LEG_JOINTS_FALLBACK)}
        kd = kd if isinstance(kd, dict) else {j: float(kd) for j in
                                              cfg.get("_leg_joints", LEG_JOINTS_FALLBACK)}
        for f in src["files"]:
            path = (ROOT / f).resolve()
            print(f"[prepare] parsing {path}")
            log = parse_csv(path, kp=kp, kd=kd)
            clips = segment_clips(log,
                                  h_min_s=args.h_min or cfg["data"]["clip_h_min_s"],
                                  h_max_s=args.h_max or cfg["data"]["clip_h_max_s"],
                                  seed=cfg["data"].get("seed", 0))
            print(f"[prepare]   {len(clips)} clips (dt={log.dt*1e3:.2f} ms)")
            all_clips.extend(clips)

    if args.max_clips or cfg["data"].get("max_clips"):
        cap = args.max_clips or cfg["data"]["max_clips"]
        rng = np.random.default_rng(cfg["data"].get("seed", 0))
        if len(all_clips) > cap:
            idx = rng.choice(len(all_clips), size=cap, replace=False)
            all_clips = [all_clips[i] for i in sorted(idx)]

    meta = {"n_clips": len(all_clips),
            "mean_len": float(np.mean([c["n"] for c in all_clips]))}
    save_clips(all_clips, meta, args.out)
    print(f"[prepare] saved {len(all_clips)} clips -> {args.out}")


LEG_JOINTS_FALLBACK = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_pitch_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_pitch_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
]


if __name__ == "__main__":
    main()
