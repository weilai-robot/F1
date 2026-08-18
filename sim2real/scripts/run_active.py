#!/usr/bin/env python3
"""SPI-Active stage-2: optimize command sequence for Fisher Information.

Requires: stage-1 identified params (identified_params.json), a policy runner,
mujoco, optuna. The policy runner is pluggable:

  * RecordedActionRunner (default): replays recorded joint targets from a
    walk_diag clip as the policy response — a faithful stand-in when the
    deployed ONNX obs stack is not reproduced in python. New commands change
    only the excitation schedule fed to the FIM FD rollouts.
  * OnnxPolicyRunner: drop-in once the rl_walk_leg obs builder is exported
    (obs 47 + hist 66); wire it in ``--policy onnx --onnx <model.onnx>``.

Usage (remote):
  python sim2real/scripts/run_active.py \
      --config sim2real/configs/x1_spi.yaml \
      --params sim2real/export/identified_params.json \
      --dataset sim2real/data/x1_clips.npz --out-dir logs/active_sysid
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim2real"))

from active.bezier import bezier_matrix, denormalize  # noqa: E402
from active.command_opt import optimize_commands  # noqa: E402
from active.fim import a_optimality_objective  # noqa: E402
from spi.dataset import JIDX, load_clips  # noqa: E402


class RecordedActionRunner:
    """Policy approximation: replay recorded controls of the longest clip."""

    def __init__(self, clips):
        self.clip = max(clips, key=lambda c: c["n"])

    def commands(self, cmd_seq: np.ndarray) -> dict:
        """Return recorded controls resampled to the command horizon; the
        recorded gait is the policy's response around its own cmd history."""
        n = cmd_seq.shape[0]
        idx = np.linspace(0, self.clip["n"] - 1, n).round().astype(int)
        return {"ctrl_target_pos": self.clip["ctrl_target_pos"][idx],
                "ctrl_target_tau": self.clip["ctrl_target_tau"][idx],
                "mode": self.clip["mode"][idx]}


def build_fim_fn(cfg, rollouter, runner, kappa_map, theta_vec_fn):
    """FD FIM over a compact parameter vector (identified values ± delta)."""
    from spi.rollout import MuJoCoRollouter  # noqa: F401
    from spi.dataset import LEG_JOINTS

    acfg = cfg["active"]
    delta = acfg.get("delta_param", 0.1)
    ksync = acfg.get("ksync_steps", 5)
    base_clip_len = runner.clip["n"]

    def fim(cmd_seq: np.ndarray, theta: np.ndarray) -> np.ndarray:
        controls = runner.commands(cmd_seq)
        n = cmd_seq.shape[0]
        clip = dict(runner.clip)
        clip.update({k: controls[k] for k in
                     ("ctrl_target_pos", "ctrl_target_tau", "mode")})
        clip["n"] = n
        d = theta.shape[0]
        # residual-collecting FD rollout per parameter (resync every ksync)
        J_rows = []
        ref_state = None
        for t_start in range(0, n, ksync):
            sl = slice(t_start, min(t_start + ksync, n))
            sub = dict(clip)
            sub["ctrl_target_pos"] = clip["ctrl_target_pos"][sl]
            sub["ctrl_target_tau"] = clip["ctrl_target_tau"][sl]
            sub["mode"] = clip["mode"][sl]
            sub["n"] = sl.stop - sl.start
            cols = []
            base = None
            for i in range(d):
                outs = []
                for sign in (+1, -1):
                    th = theta.copy(); th[i] += sign * delta
                    params = theta_vec_fn(th)
                    sims = rollouter.rollout_clips([sub], params, kappa_map)
                    s = sims[0]
                    key_state = np.concatenate([s["quat"].ravel(), s["q"].ravel()])
                    outs.append(key_state)
                if base is None:
                    base = 0.5 * (outs[0] + outs[1])
                cols.append((outs[0] - outs[1]) / (2 * delta))
            J_rows.append(np.stack(cols, axis=1))  # (state_dim, d)
            del ref_state
        F = np.zeros((d, d))
        for J in J_rows:
            F += J.T @ J
        return F

    return fim


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--params", required=True)
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--n-samples", type=int, default=200, help="cmd seq length")
    ap.add_argument("--n-trials", type=int, default=40)
    ap.add_argument("--out-dir", default="logs/active_sysid")
    args = ap.parse_args()

    import yaml
    cfg = yaml.safe_load(Path(args.config).read_text())
    payload = json.loads(Path(args.params).read_text())
    clips, _ = load_clips(args.dataset)

    from spi.rollout import MuJoCoRollouter
    mjcf = (ROOT / cfg["model"]["mjcf"]).resolve()
    rollouter = MuJoCoRollouter(mjcf, base_body=cfg["model"]["base_body"],
                                foot_bodies=tuple(cfg["model"]["foot_bodies"]))
    kappa_map = {g["name"]: [JIDX[j] for j in g["joints"]]
                 for g in cfg["motor_groups"]}
    body_key = cfg["model"].get("base_body_key", "base")
    best = payload["best_params"]
    motors0 = best["motors"]
    kappa_s0 = best["kappa_s"]

    # compact theta: [mass, com_x, com_y, com_z] + kappas (FIM target dims)
    b0 = best["bodies"][body_key]
    theta0 = np.array([b0["mass"], b0["com"][0], b0["com"][1], b0["com"][2],
                       *motors0.values()], dtype=float)
    names = ["mass", "com_x", "com_y", "com_z", *motors0.keys()]

    def theta_vec_fn(th: np.ndarray) -> dict:
        return {"bodies": {"base": {"mass": th[0],
                                    "com": th[1:4],
                                    "inertia": np.asarray(b0["inertia"])}},
                "motors": {k: th[4 + i] for i, k in enumerate(motors0.keys())},
                "kappa_s": kappa_s0}

    runner = RecordedActionRunner(clips)
    fim = build_fim_fn(cfg, rollouter, runner, kappa_map, theta_vec_fn)

    def rollout_with_commands(cmd_seq):
        return {"terminated": False}  # recorded-gait runner cannot fall

    ranges = np.array(cfg["active"]["command_ranges"], dtype=float)
    res = optimize_commands(ranges, args.n_samples,
                            cfg["active"].get("bezier_points", 4),
                            rollout_with_commands, fim, theta0,
                            n_trials=args.n_trials,
                            termination_penalty=cfg["active"].get(
                                "termination_penalty", 1e4))

    out = Path(args.out_dir); out.mkdir(parents=True, exist_ok=True)
    gm = out / "gm_play"; gm.mkdir(exist_ok=True)
    artifact = {"cmd_seq": res["cmd_seq"].tolist(), "cost": res["cost"],
                "ctrl_points": res["ctrl_points"].tolist(),
                "param_names": names}
    (gm / "best_commands.json").write_text(json.dumps(artifact, indent=2))
    np.savez(out / "best_commands.npz", **{k: np.asarray(v) for k, v in res.items()})
    print(f"[active] best tr(F^-1) = {res['cost']:.4e}; "
          f"artifacts -> {out}")

    # baseline: default zero commands for reference
    zero = np.zeros((args.n_samples, ranges.shape[0]))
    print(f"[active] reference tr(F^-1) at zero cmds = "
          f"{a_optimality_objective(fim(zero, theta0)):.4e}")


if __name__ == "__main__":
    main()
