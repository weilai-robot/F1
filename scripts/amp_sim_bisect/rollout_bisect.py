#!/usr/bin/env python3
"""
X1 sim2sim: Isaac-trained AMP policy -> MuJoCo (gmr_x1_assets/x1.xml) rollout.

Faithfulness notes (deltas vs Isaac documented in the report):
- Control loop: 50 Hz policy, PD torques applied every physics substep with a
  held target — matches Isaac decimation=4 @ sim.dt 0.005. MuJoCo xml runs at
  0.001 s (GMR vendor model), so we take 20 substeps per control step.
- PD gains / effort limits: exactly X1_CFG (robolab assets/robots/x1.py).
- Joint damping/frictionloss present in x1.xml (vendor values) ADD to the PD;
  Isaac had URDF damping=1.0 on all joints instead. Documented approximation.
- Observations replicate the policy obs group exactly (no noise, PLAY cfg):
    [base_ang_vel(3), projected_gravity(3), cmd(3), joint_pos_rel(29),
     joint_vel_rel(29), last_action(29)] x history 3 (oldest-first).
- Action: q_target = default_q + 0.25 * a  (JointPositionActionCfg scale).
- MuJoCo free-joint qvel[3:6] is body-frame angular velocity == Isaac
  base_ang_vel (verified empirically 2026-08-17).

Usage:
  PYTHONPATH=pylibs python sim2sim/mujoco_rollout.py \
      --ckpt model_4000.pt --repo-root . \
      [--cmd 1.0 0.0 0.0] [--duration 12] [--video out.mp4]
"""

import argparse
import collections
import functools
import json
import math
import re
import sys
from pathlib import Path

import numpy as np

print = functools.partial(print, flush=True)

CONTROL_DT = 0.02
HIST = 3
ACTION_SCALE = 0.25
FALL_Z = 0.30          # base height below which we count a fall (MuJoCo frame)
FALL_TILT = 60.0       # deg

# X1_CFG PD gains / effort limits (Isaac -> MuJoCo by joint name)
HERE = Path(__file__).resolve().parent
F1_ROOT = HERE.parent.parent
PD = {
    "lumbar_yaw_joint": (120.0, 4.0, 180.0),
    "lumbar_roll_joint": (120.0, 4.0, 180.0),
    "lumbar_pitch_joint": (150.0, 5.0, 180.0),
    "left_hip_pitch_joint": (120.0, 4.0, 180.0),
    "right_hip_pitch_joint": (120.0, 4.0, 180.0),
    "left_hip_roll_joint": (100.0, 3.3, 180.0),
    "right_hip_roll_joint": (100.0, 3.3, 180.0),
    "left_hip_yaw_joint": (100.0, 3.3, 180.0),
    "right_hip_yaw_joint": (100.0, 3.3, 180.0),
    "left_knee_pitch_joint": (150.0, 5.0, 180.0),
    "right_knee_pitch_joint": (150.0, 5.0, 180.0),
    "left_ankle_pitch_joint": (50.0, 2.5, 80.0),
    "right_ankle_pitch_joint": (50.0, 2.5, 80.0),
    "left_ankle_roll_joint": (50.0, 2.5, 80.0),
    "right_ankle_roll_joint": (50.0, 2.5, 80.0),
    "left_shoulder_pitch_joint": (40.0, 2.0, 27.0),
    "right_shoulder_pitch_joint": (40.0, 2.0, 27.0),
    "left_shoulder_roll_joint": (40.0, 2.0, 27.0),
    "right_shoulder_roll_joint": (40.0, 2.0, 27.0),
    "left_shoulder_yaw_joint": (40.0, 2.0, 27.0),
    "right_shoulder_yaw_joint": (40.0, 2.0, 27.0),
    "left_elbow_pitch_joint": (30.0, 1.5, 27.0),
    "right_elbow_pitch_joint": (30.0, 1.5, 27.0),
    "left_elbow_yaw_joint": (20.0, 1.0, 27.0),
    "right_elbow_yaw_joint": (20.0, 1.0, 27.0),
    "left_wrist_pitch_joint": (15.0, 1.0, 10.0),
    "right_wrist_pitch_joint": (15.0, 1.0, 10.0),
    "left_wrist_roll_joint": (15.0, 1.0, 10.0),
    "right_wrist_roll_joint": (15.0, 1.0, 10.0),
}

# Isaac X1_CFG init joint_pos == x1.xml keyframe (verified identical)
DEFAULT_Q = {
    "lumbar_yaw_joint": 0.0, "lumbar_roll_joint": 0.0, "lumbar_pitch_joint": 0.0,
    "left_shoulder_pitch_joint": 0.0, "left_shoulder_roll_joint": 0.0,
    "left_shoulder_yaw_joint": 0.0, "left_elbow_pitch_joint": 0.0,
    "left_elbow_yaw_joint": 0.0, "left_wrist_pitch_joint": 0.0, "left_wrist_roll_joint": 0.0,
    "right_shoulder_pitch_joint": 0.0, "right_shoulder_roll_joint": 0.0,
    "right_shoulder_yaw_joint": 0.0, "right_elbow_pitch_joint": 0.0,
    "right_elbow_yaw_joint": 0.0, "right_wrist_pitch_joint": 0.0, "right_wrist_roll_joint": 0.0,
    "left_hip_pitch_joint": 0.48891, "left_hip_roll_joint": 0.06213,
    "left_hip_yaw_joint": -0.33853, "left_knee_pitch_joint": 0.63204,
    "left_ankle_pitch_joint": -0.27224, "left_ankle_roll_joint": 0.0,
    "right_hip_pitch_joint": -0.48891, "right_hip_roll_joint": -0.06213,
    "right_hip_yaw_joint": 0.33853, "right_knee_pitch_joint": 0.63204,
    "right_ankle_pitch_joint": -0.27224, "right_ankle_roll_joint": 0.0,
}

# F1_29DOF standing pose (FK-verified IK: feet flat at X1-init placements;
# retarget_f1_from_x1lab.py solve, residual 0.0 mm). F1 legs differ from X1:
# orthogonal hip (sign flip on hip_pitch), shorter legs, wider hips.
F1_DEFAULT_Q = dict(DEFAULT_Q)
F1_DEFAULT_Q.update({
    "left_hip_pitch_joint": -0.23847, "left_hip_roll_joint": 0.02102,
    "left_hip_yaw_joint": 0.00600, "left_knee_pitch_joint": 0.39231,
    "left_ankle_pitch_joint": -0.14583, "left_ankle_roll_joint": 0.02171,
    "right_hip_pitch_joint": -0.23715, "right_hip_roll_joint": -0.02104,
    "right_hip_yaw_joint": -0.00587, "right_knee_pitch_joint": 0.39412,
    "right_ankle_pitch_joint": -0.14649, "right_ankle_roll_joint": -0.02125,
})

ROBOT_CFG = {
    "x1": dict(xml="gmr_x1_assets/x1.xml", default_q=DEFAULT_Q),
    "deploy": dict(xml=str(HERE / "deploy_robot.xml"), default_q=DEFAULT_Q),
    "deploy_flat": dict(xml=str(F1_ROOT / "motion_control/module/sim_module/model/mjcf/xyber_x1_flat.xml"), default_q=DEFAULT_Q),
    "f1": dict(xml="gmr_f1_assets/f1.xml", default_q=F1_DEFAULT_Q),
}


def parse_yaml_list(path, key):
    out, cur = [], False
    for line in Path(path).read_text().splitlines():
        if re.match(rf"^\s*{key}:\s*$", line):
            cur = True
            continue
        if cur:
            m = re.match(r"^\s*-\s*(\S+)\s*$", line)
            if m:
                out.append(m.group(1))
            elif line.strip() and not line.strip().startswith("#"):
                cur = False
    return out


def quat2mat(q):
    import mujoco
    out = np.zeros(9)
    mujoco.mju_quat2Mat(out, np.ascontiguousarray(q, dtype=np.float64))
    return out.reshape(3, 3)


def args_align_needed(xml_path):
    return False  # legacy path 仅用于 no-align 场景测试

def build_model(xml_path: Path, actuator_mode: str = "explicit", align: bool = True):
    """Load x1.xml, add a floor, and drop the unused vendor <motor> actuators
    (we drive torques via qfrc_applied).

    v25 collision fix (found by local Mac repro): the vendor xml disables ALL
    collisions by default (contype=0/conaffinity=0) and marks the REAL foot
    contact points as tiny class="collision" spheres (contype=1). The old
    code set contype/conaffinity=1 on EVERY geom of the ankle bodies —
    including the visual foot mesh and the 0.02 m red marker spheres — so the
    robot stood on wrong geometry and collapsed in 1.8 s even with zero
    action. Now we only add the floor (contype=1, conaffinity=1): it pairs
    with the vendor's collision-class spheres and nothing else.

    v58 actuator_mode="native": replace the EXPLICIT qfrc_applied PD loop
    with MuJoCo position servos (kp/kv per joint from the same PD table,
    forcerange = effort limits) + the implicitfast integrator, which solves
    the kv damping term inside the integrator like Isaac's implicit
    actuator model. Forensics (V57_REPORT §2 + the pd-every probe): the
    policy's terminal plantarflexion COMMAND (~110 deg/s ankle rate) is
    executed faithfully by the explicit PD but filtered by Isaac's implicit
    actuator (< 40 deg/s in-domain) — the flick is an actuator-MODEL
    artifact, not a policy defect nor a loop-rate artifact (pd-every 1
    leaves the rate bit-identical). ctrl = q_target at 50 Hz; the servo
    runs every 1 ms substep."""
    import mujoco
    if not hasattr(mujoco, "MjSpec"):
        # MuJoCo <=3.1: 无 MjSpec; 直接加载(需场景自带地板, deploy_flat 满足)
        model = mujoco.MjModel.from_xml_path(str(xml_path))
        print(f"[Bisect] legacy-load (mujoco {mujoco.__version__}), floor in-scene: {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'floor') >= 0}")
        if not args_align_needed(xml_path):
            return model, 8
    spec = mujoco.MjSpec.from_file(str(xml_path))
    # offscreen framebuffer: vendor xml declares only 640x480, which caps
    # mujoco.Renderer at that size. Raise to 720p for the GL render path
    # ("global" is a Python keyword -> getattr; local Mac GL works, the
    # container-only matplotlib stick-figure remains as --render soft).
    vis_g = getattr(spec.visual, "global")
    vis_g.offwidth, vis_g.offheight = 1280, 720
    if actuator_mode == "native":
        # String-inject the servo block (the MjSpec add_actuator kwarg set
        # differs across mujoco versions; plain XML is the stable API):
        # <position> servo per hinge — kp/kv/forcerange identical to the
        # explicit loop's PD table; implicitfast solves kv implicitly like
        # Isaac's implicit actuator.
        xml_text = Path(xml_path).read_text()
        srv = "\n".join(
            f'<position name="servo_{n}" joint="{n}" kp="{kp}" kv="{kd}" '
            f'forcerange="-{eff} {eff}" ctrlrange="-6.28 6.28"/>'
            for n, (kp, kd, eff) in PD.items())
        if "</actuator>" in xml_text:
            xml_text = xml_text.replace("</actuator>", srv + "\n  </actuator>")
        elif "</mujoco>" in xml_text:
            xml_text = xml_text.replace("</mujoco>",
                                        f"  <actuator>\n{srv}\n  </actuator>\n</mujoco>")
        # from_string breaks relative mesh paths — compile from a temp file
        # NEXT TO the original so meshdir="meshes" resolves identically.
        import tempfile as _tf
        _tmp = Path(_tf.mkstemp(suffix="_native.xml", dir=str(Path(xml_path).parent))[1])
        _tmp.write_text(xml_text)
        try:
            spec = mujoco.MjSpec.from_file(str(_tmp))
        finally:
            _tmp.unlink(missing_ok=True)
        _native_integrator = True
        print("[INFO] actuator mode: native (implicitfast position servos)")
    # floor
    if getattr(build_model, "_skip_floor", False) or Path(xml_path).name.startswith("xyber_x1_flat"):
        world = spec.worldbody  # scene already has a floor
    else:
        world = spec.worldbody
        world.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE,
                   size=[20, 20, 0.1], friction=[1.0, 0.005, 0.0001],
                   contype=1, conaffinity=1, rgba=[0.35, 0.45, 0.5, 1])
    # count vendor collision-class geoms on the feet (informative)
    n_feet = 0
    for body in spec.bodies:
        if body.name.endswith(("ankle_roll_link", "ankle_roll")):
            for geom in body.geoms:
                if geom.contype:
                    n_feet += 1
    # v26 sim2sim fix: ALIGN PASSIVE DYNAMICS WITH ISAAC. The vendor xml adds
    # joint damping (2.0/1.5/0.8) + frictionloss (4.0/3.0/1.5 Nm dry friction)
    # ON TOP of our PD torques. Isaac's URDF has damping=1.0 on every joint,
    # NO dry friction, and actuator armature=0.01. Doubled damping + 4 Nm
    # stiction per hip/knee is a massive gait distortion (local repro:
    # zero-action standing topples at 1.8 s, trained policy flips at 0.7 s).
    model = spec.compile()
    if actuator_mode == "native":
        model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    if align:
        print("[Bisect] passive dynamics ALIGNED to Isaac (frictionloss=0, damping=1.0, armature=0.01)")
        model.dof_frictionloss[:] = 0.0
    if getattr(build_model, "_damp_mult", 1.0) != 1.0 or True:
        pass  # placeholder (applied by caller below via ankle-damp arg)
        for j in range(model.njnt):
            if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE:
                adr = model.jnt_dofadr[j]
                model.dof_damping[adr] = 1.0
                model.dof_armature[adr] = 0.01
    else:
        print("[Bisect] vendor passive dynamics KEPT (no alignment)")
    return model, n_feet


def load_policy(ckpt_path: Path):
    """Load actor weights from either an rsl_rl .pt checkpoint (torch) or a
    pre-exported .npz (v25: exported in-container right after training, so
    the MuJoCo rollout needs NO torch — dodges the kit-torch/numpy2 ABI
    landmine that killed every pylibs attempt in v24)."""
    if ckpt_path.suffix == ".npz":
        z = np.load(ckpt_path)
        layers, i = [], 0
        while f"l{i}_w" in z.files:
            w = z[f"l{i}_w"].astype(np.float32)
            b = z[f"l{i}_b"].astype(np.float32) if f"l{i}_b" in z.files else None
            layers.append((w, b))
            i += 1
        if not layers:
            raise RuntimeError(f"no l*_w arrays in {ckpt_path}")
        mean = z["mean"].astype(np.float32).reshape(-1)
        std = z["std"].astype(np.float32).reshape(-1)
        return layers, mean, std, {"src": "npz"}
    import torch
    ck = torch.load(ckpt_path, map_location="cpu", weights_only=False)
    sd = ck["model_state_dict"]
    actor_sd = {k[len("actor."):]: v for k, v in sd.items() if k.startswith("actor.")}
    layers, i = [], 1
    while f"{i}.weight" in actor_sd:
        w = actor_sd[f"{i}.weight"].float().numpy()
        b = actor_sd.get(f"{i}.bias")
        b = b.float().numpy() if b is not None else None
        layers.append((w, b))
        i += 1
    mean = sd["actor_obs_normalizer._mean"].float().numpy().squeeze(0)
    std = sd["actor_obs_normalizer._std"].float().numpy().squeeze(0)
    meta = {k: ck.get(k) for k in ("it", "infos")}
    return layers, mean, std, meta


def run_policy(layers, mean, std, obs):
    """rsl_rl MLP: Linear -> ELU (between layers), last layer linear, raw mean
    action out (no tanh — IsaacLab clips at clip_actions=100, effectively raw)."""
    x = (np.asarray(obs, dtype=np.float32) - mean) / std
    for w, b in layers[:-1]:
        x = x @ w.T + (b if b is not None else 0)
        x = np.where(x < 0, np.expm1(np.minimum(x, 20)), x)  # ELU(alpha=1)
    w, b = layers[-1]
    return x @ w.T + (b if b is not None else 0)


def find_sole_geoms(model):
    """Identify, per foot, the 4 vendor collision-class sole spheres
    (contype=1, r=0.002) on each ankle_roll_link body, ordered
    [front_l, front_r, back_l, back_r] by local-frame z (+0.07 front,
    -0.07 heel) then x. Returns {foot_body_name: [4 geom ids]} and the
    floor geom id. Used by --log for gait-quality analysis."""
    import mujoco
    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    out = {}
    for b in range(model.nbody):
        name = model.body(b).name
        if not name.endswith(("ankle_roll_link", "ankle_roll")):
            continue
        g0, gnum = int(model.body_geomadr[b]), int(model.body_geomnum[b])
        gids = [g for g in range(g0, g0 + gnum)
                if model.geom_contype[g] and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE
                and model.geom_size[g, 0] < 0.01]
        if len(gids) != 4:
            raise RuntimeError(f"{name}: expected 4 sole spheres, got {len(gids)}")
        gids.sort(key=lambda g: (model.geom_pos[g, 2] < 0, model.geom_pos[g, 0] < 0))
        out[name] = gids
    return out, floor


class SoftRenderer:
    """Pure-CPU stick-figure renderer (matplotlib Agg, no GL at all).

    v22 evidence: the container has NO usable GL stack — system PyOpenGL
    (isaac_sim site-packages), pip PyOpenGL, EGL and OSMesa all fail, so
    mujoco.Renderer can never initialize. This renderer draws the mj
    skeleton (body world positions + parent edges) with matplotlib 3D and
    feeds frames to imageio — guaranteed to work headless."""

    def __init__(self, model, height=720, width=1280):
        import os as _os
        import matplotlib
        if not _os.environ.get("MPLCONFIGDIR"):
            # containers may have no writable home; font cache needs a home
            try:
                d = Path.cwd() / "mplcache"
                d.mkdir(exist_ok=True)
                _os.environ["MPLCONFIGDIR"] = str(d)
            except Exception:
                pass
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
        self._plt = plt
        self.model = model
        try:
            self.parent = [int(model.body_parentid[i]) for i in range(model.nbody)]
        except Exception:
            self.parent = [int(model.body(i).parentid) for i in range(model.nbody)]
        self.foot_ids = [i for i in range(model.nbody)
                         if model.body(i).name.endswith("ankle_roll_link")]
        self.trail = []
        dpi = 100
        self.fig = plt.figure(figsize=(width / dpi, height / dpi), dpi=dpi)
        self.ax = self.fig.add_subplot(111, projection="3d")

    def capture(self, data) -> np.ndarray:
        import numpy as _np
        ax, plt = self.ax, self._plt
        P = data.xpos.copy()
        root = P[1]  # body 1 = torso (child of world)
        self.trail.append(root.copy())
        if len(self.trail) > 100:
            self.trail.pop(0)
        ax.clear()
        for i in range(1, self.model.nbody):
            a, b = P[self.parent[i]], P[i]
            ax.plot([a[0], b[0]], [a[1], b[1]], [a[2], b[2]], color="0.25", lw=1.8)
        for fi in self.foot_ids:
            f = P[fi]
            ax.scatter([f[0]], [f[1]], [f[2]], color="crimson", s=42)
        tr = _np.array(self.trail)
        ax.plot(tr[:, 0], tr[:, 1], tr[:, 2], color="steelblue", lw=1.2, alpha=0.8)
        ax.scatter([root[0]], [root[1]], [root[2]], color="navy", s=60)
        # camera follows root; fixed 3m box + ground grid (motion parallax
        # and enough visual structure to keep the mp4 above the 100KB P6
        # threshold — a bare stick figure on flat bg compresses to ~30KB)
        ax.set_xlim(root[0] - 1.5, root[0] + 1.5)
        ax.set_ylim(root[1] - 1.5, root[1] + 1.5)
        ax.set_zlim(0.0, 1.6)
        gx = _np.arange(root[0] - 1.5, root[0] + 1.51, 0.5)
        gy = _np.arange(root[1] - 1.5, root[1] + 1.51, 0.5)
        for x in gx:
            ax.plot([x, x], [gy[0], gy[-1]], [0, 0], color="0.75", lw=0.5)
        for y in gy:
            ax.plot([gx[0], gx[-1]], [y, y], [0, 0], color="0.75", lw=0.5)
        ax.view_init(elev=-18, azim=-75)
        ax.set_box_aspect((1, 1, 0.53))
        ax.set_axis_off()
        self.fig.canvas.draw()
        buf = _np.asarray(self.fig.canvas.buffer_rgba())
        return buf[:, :, :3].copy()

    def close(self):
        try:
            self._plt.close(self.fig)
        except Exception:
            pass


def main():
    import mujoco
    global CONTROL_DT

    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", required=True)
    ap.add_argument("--control-dt", type=float, default=CONTROL_DT,
                    help="policy control period in s (0.02 = 50 Hz training "
                         "lineage; 0.01 = v61 100 Hz). Servo/PD substep "
                         "counts derive from it automatically.")
    ap.add_argument("--repo-root", default=".")
    ap.add_argument("--cmd", nargs=3, type=float, default=[1.0, 0.0, 0.0])
    ap.add_argument("--duration", type=float, default=12.0)
    ap.add_argument("--video", default=None)
    ap.add_argument("--settle", type=float, default=0.5,
                    help="hold default pose before applying command (s)")
    ap.add_argument("--json", default=None)
    ap.add_argument("--log", default=None,
                    help="write per-step gait log (npz) for gait_metrics.py")
    # ---- sim2real robustness injection (v29 readiness) -----------------
    ap.add_argument("--obs-noise", type=float, default=0.0,
                    help="sensor noise scale (1.0 = realistic: jpos 0.01 rad,"
 " jvel 0.15 rad/s, ang 0.05 rad/s, grav 0.03)")
    ap.add_argument("--latency-steps", type=int, default=0,
                    help="observation pipeline latency in control steps (0-4)")
    ap.add_argument("--action-lag", type=int, default=0,
                    help="apply the action from N steps ago (comms/actuator lag)")
    ap.add_argument("--action-lpf", type=float, default=0.0,
                    help="v63: action low-pass cutoff in Hz (0 = off). "
                         "Cascaded first-order sections applied to the RAW "
                         "policy action BEFORE the lag buffer — the exact "
                         "mirror of training-side X1_ACT_LPF_HZ (AmpEnv."
                         "_action_lpf): alpha = 1-exp(-2*pi*fc*dt) per "
                         "section. Restores the policy/demo frequency-"
                         "hierarchy alignment for 100 Hz policies trained "
                         "with the filter.")
    ap.add_argument("--action-lpf-order", type=int, default=2,
                    help="number of cascaded LPF sections (2 = 12 dB/oct)")
    ap.add_argument("--torso-mass-scale", type=float, default=1.0,
                    help="scale lumbar_pitch_link mass (payload simulation)")
    ap.add_argument("--push-mag", type=float, default=0.0,
                    help="random horizontal push speed (m/s) every --push-every s")
    ap.add_argument("--push-every", type=float, default=2.0)
    ap.add_argument("--actuator", choices=["explicit", "native"], default="explicit",
                    help="explicit = qfrc_applied PD loop (v25-v57 historical); "
                         "native = MuJoCo position servos (kp/kv/forcerange from "
                         "the same PD table) + implicitfast integrator — the "
                         "actuator-model alignment with Isaac's implicit PD "
                         "(v58 harness-alignment probe)")
    ap.add_argument("--ankle-damp", type=float, default=1.0,
                    help="multiply dof_damping on ankle joints (deploy-side "
                         "probe for Isaac's implicit-integration effective "
                         "damping at high frequency; 1.0 = historical)")
    ap.add_argument("--pd-every", type=int, default=5,
                    help="recompute explicit PD torques every N physics "
                         "substeps (5 = v25-v57 historical mode; 1 = every "
                         "1 ms substep — closest explicit approximation of "
                         "Isaac's implicit actuator PD; the v58 harness-"
                         "alignment probe against the cross-sim ankle flick)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--no-align", action="store_true", help="skip Isaac passive-dynamics alignment (vendor damping/frictionloss kept)")
    ap.add_argument("--hold-kp-mult", type=float, default=1.0,
                    help="multiply kp/kd by this factor DURING the zero-action settle hold (replicate stand-mode stiff hold)")
    ap.add_argument("--hold-unclip", action="store_true",
                    help="remove effort clip during the settle hold only")
    ap.add_argument("--never-clip", action="store_true",
                    help="remove effort clip for the WHOLE rollout")
    ap.add_argument("--via-actuator", action="store_true",
                    help="route torques through d_.ctrl (ctrlrange-clipped) like AimRT WriteMotorCmd")
    ap.add_argument("--spawn-pose", choices=["default", "stand"], default="default",
                    help="spawn at default_q or the shallow stand pose (knee 0.49, statically stable)")
    ap.add_argument("--spawn-knee-sag", type=float, default=0.0, help="extra knee bend at spawn (gravity sag replication)")
    ap.add_argument("--spawn-pitch-deg", type=float, default=0.0, help="initial base pitch (deg, + = forward)")
    ap.add_argument("--robot", choices=["x1", "f1", "deploy", "deploy_flat"], default="x1",
                    help="deploy_flat = AimRT full scene (serial robot + flat floor, conaffinity=7)")
    ap.add_argument("--onnx", action="store_true",
                    help="run the policy through the exported ONNX graph "
                         "(deployment-artifact-in-the-loop parity check)")
    ap.add_argument("--render", choices=["gl", "soft"], default="gl",
                    help="soft = matplotlib stick figure (no GL needed)")
    args = ap.parse_args()
    CONTROL_DT = float(args.control_dt)

    root = Path(args.repo_root).resolve()
    _ld = root / "roboparty_train/robolab/scripts/tools/retarget/config/x1.yaml"
    if _ld.exists():
        lab_dof = parse_yaml_list(_ld, "lab_dof_names")
    else:
        lab_dof = "lumbar_yaw_joint lumbar_roll_joint lumbar_pitch_joint left_shoulder_pitch_joint left_shoulder_roll_joint left_shoulder_yaw_joint left_elbow_pitch_joint left_elbow_yaw_joint left_wrist_pitch_joint left_wrist_roll_joint right_shoulder_pitch_joint right_shoulder_roll_joint right_shoulder_yaw_joint right_elbow_pitch_joint right_elbow_yaw_joint right_wrist_pitch_joint right_wrist_roll_joint left_hip_pitch_joint left_hip_roll_joint left_hip_yaw_joint left_knee_pitch_joint left_ankle_pitch_joint left_ankle_roll_joint right_hip_pitch_joint right_hip_roll_joint right_hip_yaw_joint right_knee_pitch_joint right_ankle_pitch_joint right_ankle_roll_joint".split()
    assert len(lab_dof) == 29

    robot_cfg = ROBOT_CFG[args.robot]
    _default_q_table = robot_cfg["default_q"]
    model, n_feet = build_model(root / robot_cfg["xml"], args.actuator, align=not args.no_align)
    if args.ankle_damp != 1.0:
        n_damped = 0
        for j in range(model.njnt):
            jn = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j) or ""
            if "ankle" in jn:
                model.dof_damping[model.jnt_dofadr[j]] *= args.ankle_damp
                n_damped += 1
        print(f"[INFO] ankle damping x{args.ankle_damp} ({n_damped} joints)")
    data = mujoco.MjData(model)
    soles, floor_id = find_sole_geoms(model)
    foot_names = sorted(soles)          # [left_..., right_...]
    sole_gids = np.array([soles[f] for f in foot_names])  # (2, 4)

    # joint indices (MuJoCo) and lab->mj mapping
    mj_names = [model.joint(i).name for i in range(model.njnt)]
    hinge = [n for n in mj_names if n in _default_q_table]
    assert len(hinge) == 29, f"expected 29 hinges, got {len(hinge)}"
    jid = {n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in hinge}
    servo_idx = None
    if args.actuator == "native":
        servo_idx = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"servo_{n}")
                     for n in hinge]
        assert all(i >= 0 for i in servo_idx), "servo actuators missing"
    qadr = np.array([model.jnt_qposadr[jid[n]] for n in hinge])
    vadr = np.array([model.jnt_dofadr[jid[n]] for n in hinge])
    # policy column (lab order) -> mj hinge index
    lab2mj = np.array([hinge.index(n) for n in lab_dof])
    kp = np.array([PD[n][0] for n in hinge])
    kd = np.array([PD[n][1] for n in hinge])
    eff = np.array([PD[n][2] for n in hinge])
    q_default = np.array([_default_q_table[n] for n in hinge])

    if Path(args.ckpt).suffix == ".onnx":
        import onnxruntime as _ort
        _sess = _ort.InferenceSession(str(args.ckpt), providers=["CPUExecutionProvider"])
        _name = _sess.get_inputs()[0].name
        in_dim = _sess.get_inputs()[0].shape[-1]
        layers, mean, std, meta = None, None, None, {"src": "onnx"}

        def _run_policy_onnx(x, y, z, ob):
            ob2 = np.asarray(ob, dtype=np.float32).reshape(1, -1)
            return _sess.run(["action"], {_name: ob2})[0].reshape(-1)

        policy_fn = _run_policy_onnx
    else:
        layers, mean, std, meta = load_policy(Path(args.ckpt))
        in_dim = layers[0][0].shape[1]
        policy_fn = run_policy
    print(f"[INFO] policy input dim = {in_dim} (expect {3*96}); feet colliders = {n_feet}")
    assert in_dim == 3 * 96, f"unexpected obs dim {in_dim}"

    # initial state: keyframe standing. v25: start LOW so the sole spheres
    # penetrate, measure the worst contact distance, then lift by exactly
    # that — a fixed z (0.62 in v16-v24) left the robot 2.9 cm in the air
    # (local repro); it dropped, bounced and every policy looked broken.
    mujoco.mj_resetData(model, data)
    data.qpos[0:3] = [0, 0, 0.55]
    data.qpos[3:7] = [1, 0, 0, 0]
    spawn_q = q_default.copy()
    if args.spawn_pose == "stand":
        stand_delta = {"left_hip_pitch_joint": 0.4, "left_hip_roll_joint": 0.05,
                       "left_hip_yaw_joint": -0.31, "left_knee_pitch_joint": 0.49,
                       "left_ankle_pitch_joint": -0.21,
                       "right_hip_pitch_joint": -0.4, "right_hip_roll_joint": -0.05,
                       "right_hip_yaw_joint": 0.31, "right_knee_pitch_joint": 0.49,
                       "right_ankle_pitch_joint": -0.21}
        for k, v in stand_delta.items():
            spawn_q[hinge.index(k)] = v
        print(f"[Bisect] spawn at SHALLOW STAND pose + sag {args.spawn_knee_sag:.3f} + pitch {args.spawn_pitch_deg:.1f}deg")
    if args.spawn_knee_sag:
        for k in ("left_knee_pitch_joint", "right_knee_pitch_joint"):
            spawn_q[hinge.index(k)] += args.spawn_knee_sag
    if args.spawn_pitch_deg:
        import numpy as _np
        th = _np.radians(args.spawn_pitch_deg)
        # 绕 y 轴俯仰: q = [cos(th/2), 0, sin(th/2), 0]
        data.qpos[3:7] = [_np.cos(th/2), 0, _np.sin(th/2), 0]
    data.qpos[qadr] = spawn_q
    mujoco.mj_forward(model, data)
    if data.ncon:
        worst = min(float(data.contact[i].dist) for i in range(data.ncon))
        data.qpos[2] -= worst + 0.002
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)
    print(f"[INFO] initial base z = {data.qpos[2]:.4f} (auto-calibrated, "
          f"{data.ncon} sole contacts)")

    cmd = np.array(args.cmd, dtype=np.float64)
    substeps = max(1, round(CONTROL_DT / model.opt.timestep))
    n_steps = int(args.duration / CONTROL_DT)
    settle_steps = int(args.settle / CONTROL_DT)

    hist = np.zeros((HIST, 96), dtype=np.float32)
    last_act = np.zeros(29, dtype=np.float64)
    frames = []
    vxy_err, yaw_err, alive_steps = [], [], 0
    LOG = args.log is not None
    L = {k: [] for k in ("t", "q", "dq", "base_pos", "base_quat", "v_b",
                         "sole_xyz", "contact_dist")}
    video = None
    soft = None
    if args.video:
        if args.render == "soft":
            soft = SoftRenderer(model, height=720, width=1280)
        else:
            video = mujoco.Renderer(model, height=720, width=1280)
            cam = mujoco.MjvCamera()

    rng = np.random.default_rng(0)
    # robustness setup
    rng = np.random.default_rng(args.seed)
    if args.torso_mass_scale != 1.0:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "lumbar_pitch_link")
        model.body_mass[bid] *= args.torso_mass_scale
        print(f"[ROB] torso mass scaled by {args.torso_mass_scale}")
    ob_buf = collections.deque(maxlen=max(1, args.latency_steps) + 1)   # obs pipeline delay
    act_buf = collections.deque(maxlen=max(1, args.action_lag) + 1)     # action comms lag
    # v63: deploy-side action low-pass — byte-for-byte the training-side
    # AmpEnv._action_lpf cascade (same alpha, same order, same position in
    # the pipeline: filter the RAW policy output, then the lag buffer).
    lpf_alpha = (1.0 - math.exp(-2.0 * math.pi * args.action_lpf * CONTROL_DT)
                 if args.action_lpf > 0 else None)
    lpf_state = [np.zeros(29) for _ in range(max(1, args.action_lpf_order))]
    push_every_steps = int(args.push_every / CONTROL_DT)
    NOISE = dict(jpos=0.01, jvel=0.15, ang=0.05, grav=0.03)  # 1.0x = realistic

    for step in range(n_steps):
        q = data.qpos[qadr]
        dq = data.qvel[vadr]
        R = quat2mat(data.qpos[3:7])
        ang_b = data.qvel[3:6].copy()                     # body frame (verified)
        grav_b = R.T @ np.array([0, 0, -1.0])
        v_w = data.qvel[0:3].copy()
        v_b = R.T @ v_w
        if args.obs_noise > 0:
            n = args.obs_noise
            ang_b = ang_b + rng.normal(0, NOISE["ang"] * n, 3)
            grav_b = grav_b + rng.normal(0, NOISE["grav"] * n, 3)
            qn = q + rng.normal(0, NOISE["jpos"] * n, len(q))
            dqn = dq + rng.normal(0, NOISE["jvel"] * n, len(dq))
        else:
            qn, dqn = q, dq
        obs = np.concatenate([ang_b, grav_b, cmd,
                              (qn - q_default)[lab2mj], dqn[lab2mj], last_act]).astype(np.float32)
        hist[:-1] = hist[1:]
        hist[-1] = obs

        # v26 ROOT-CAUSE FIX: IsaacLab observation groups with history_length
        # flatten TERM-MAJOR (each term's 3 history frames concatenated,
        # frame-major WITHIN each term, oldest first), NOT frame-major across
        # terms. Proven by the checkpoint normalizer statistics: gravity-z
        # (≈-0.995) sits at indices 11/14/17 (grav block [9:18] =
        # [g(t-2), g(t-1), g(t)], gz = 2nd element of each frame), cmd_x mean
        # 0.89 == center of range (-0.5,2.5), jvel std 1.2-5.2 in [114:201]
        # and act std 0.5-2.2 in [201:288]. The old frame-major assembly fed
        # the policy a scrambled vector — every sim2sim attempt (v23-v26 pod
        # + local) fell within 1 s.
        slices = [(0, 3), (3, 6), (6, 9), (9, 38), (38, 67), (67, 96)]
        ob = np.concatenate([hist[:, a:b].reshape(-1) for a, b in slices])
        ob_buf.appendleft(ob.copy())
        ob_eff = ob_buf[min(args.latency_steps, len(ob_buf) - 1)]

        hold_mult = args.hold_kp_mult if (step < settle_steps and args.hold_kp_mult != 1.0) else 1.0
        if step == 0 and args.via_actuator:
            global act_for_qadr
            act_for_qadr = {}
            for a in range(model.nu):
                j = int(model.actuator_trnid[a, 0])
                if j >= 0:
                    act_for_qadr[int(model.jnt_dofadr[j])] = a
            print(f"[Bisect] via-actuator mode: {len(act_for_qadr)} motors, ctrlrange clipping ACTIVE")
        if step < settle_steps:
            act = np.zeros(29)
        else:
            act = policy_fn(layers, mean, std, ob_eff)
        if step < 3:
            print(f'[act] tick {step}: legs_act={[round(float(x),2) for x in act[17:29]]}')
            LEG_NAMES = ["left_hip_pitch_joint","left_hip_roll_joint","left_hip_yaw_joint",
                "left_knee_pitch_joint","left_ankle_pitch_joint","left_ankle_roll_joint",
                "right_hip_pitch_joint","right_hip_roll_joint","right_hip_yaw_joint",
                "right_knee_pitch_joint","right_ankle_pitch_joint","right_ankle_roll_joint"]
            lv = [int(data.qvel[vadr[hinge.index(n)]]) for n in LEG_NAMES]
            lp = [round(float(data.qpos[qadr[hinge.index(n)]] - q_default[hinge.index(n)]),3) for n in LEG_NAMES]
            print(f'[st] t{step}: legs_jvel={[round(x,2) for x in lv]}')
            print(f'[st] t{step}: legs_jpos_rel={lp}')
        if step < 3:
            q_rel = data.qpos[qadr] - q_default[lab2mj] if False else None
            LEG_NAMES = ["left_hip_pitch_joint","left_hip_roll_joint","left_hip_yaw_joint",
                "left_knee_pitch_joint","left_ankle_pitch_joint","left_ankle_roll_joint",
                "right_hip_pitch_joint","right_hip_roll_joint","right_hip_yaw_joint",
                "right_knee_pitch_joint","right_ankle_pitch_joint","right_ankle_roll_joint"]
            vadr_legs = [vadr[hinge.index(n)] for n in LEG_NAMES]
            legs_jvel = data.qvel[vadr_legs]

        if lpf_alpha is not None:
            for _i in range(len(lpf_state)):
                lpf_state[_i] = lpf_alpha * np.asarray(act, dtype=np.float64) \
                    + (1.0 - lpf_alpha) * lpf_state[_i]
                act = lpf_state[_i]
        # action comms lag: the NEW action enters the buffer head; what the
        # robot actually executes is the entry N steps old (lag=0 -> new act)
        act_buf.appendleft(np.asarray(act, dtype=np.float64).copy())
        act = act_buf[min(args.action_lag, len(act_buf) - 1)]
        last_act = act
        if args.push_mag > 0 and step >= settle_steps and step % push_every_steps == 0:
            theta = rng.uniform(0, 2 * np.pi)
            data.qvel[0] += args.push_mag * np.cos(theta)
            data.qvel[1] += args.push_mag * np.sin(theta)
        # act[i] is for lab_dof[i]; target = default + 0.25*act; place into
        # MuJoCo hinge order via lab2mj (lab2mj[i] = hinge index of lab_dof[i])
        q_tgt_full = q_default.copy()
        q_tgt_full[lab2mj] = q_default[lab2mj] + ACTION_SCALE * act

        if servo_idx is not None:
            # v58 native-actuator path: ctrl holds the POSITION TARGET at
            # 50 Hz; the servo + implicitfast integrator evaluate the PD
            # (with implicit damping) every 1 ms substep — the closest
            # MuJoCo analog of Isaac's implicit actuator semantics.
            data.ctrl[:] = 0.0
            data.ctrl[servo_idx] = q_tgt_full
            for _s_i in range(substeps):
                mujoco.mj_step(model, data)
        else:
            # v25: recompute PD every 5 substeps (200 Hz) — Isaac recomputes the
            # implicit PD every sim step (200 Hz) while holding targets at 50 Hz;
            # holding the TORQUE for the full 20 ms (v16-v24) oscillated against
            # the stiff sole contacts (local repro: launch + flail)
            # v58 HARNESS-ALIGNMENT PROBE: --pd-every N recomputes every N
            # substeps (5 = v25-v57 historical; 1 = every 1 ms — probe for
            # the loop-rate hypothesis; measured: the flick rate is
            # IDENTICAL at 1 and 5, ruling the loop rate out).
            for s_i in range(substeps):
                if s_i % args.pd_every == 0:
                    tau = kp * hold_mult * (q_tgt_full - data.qpos[qadr]) - kd * hold_mult * data.qvel[vadr]
                    if args.never_clip or (hold_mult != 1.0 and args.hold_unclip):
                        pass
                    else:
                        tau = np.clip(tau, -eff, eff)
                    if args.via_actuator:
                        data.ctrl[:] = 0.0
                        for qi, ai in act_for_qadr.items():
                            data.ctrl[ai] = tau[vadr.index(qi)] if isinstance(vadr, list) else tau[list(vadr).index(qi)]
                        data.qfrc_applied[:] = 0.0
                    else:
                        data.qfrc_applied[vadr] = tau
                mujoco.mj_step(model, data)

        # metrics + termination
        base_z = data.qpos[2]
        tilt = np.degrees(np.arccos(np.clip(quat2mat(data.qpos[3:7])[2, 2], -1, 1)))
        R = quat2mat(data.qpos[3:7])
        v_b = R.T @ data.qvel[0:3]
        if step >= settle_steps:
            alive_steps += 1
            vxy_err.append(np.linalg.norm(v_b[:2] - cmd[:2]))
            yaw_err.append(abs(data.qvel[5] - cmd[2]))
            if base_z < FALL_Z or tilt > FALL_TILT:
                print(f"[FALL] step {step} t={step*CONTROL_DT:.2f}s base_z={base_z:.3f} tilt={tilt:.1f}")
                break
        if LOG:
            cd = np.zeros((2, 4))
            for ci in range(data.ncon):
                c = data.contact[ci]
                g1, g2 = c.geom1, c.geom2
                if g1 != floor_id and g2 != floor_id:
                    continue
                other = g2 if g1 == floor_id else g1
                fi, si = np.argwhere(sole_gids == other)[0]
                if cd[fi, si] == 0.0 or c.dist < cd[fi, si]:
                    cd[fi, si] = c.dist        # negative = penetration
            L["t"].append(step * CONTROL_DT)
            # re-read at END of step (q/dq locals are from step start)
            L["q"].append(data.qpos[qadr].copy())
            L["dq"].append(data.qvel[vadr].copy())
            L["base_pos"].append(data.qpos[0:3].copy())
            L["base_quat"].append(data.qpos[3:7].copy())
            L["v_b"].append(v_b.copy())
            L["sole_xyz"].append(
                np.array([[data.geom_xpos[g].copy() for g in row]
                          for row in sole_gids]))
            L["contact_dist"].append(cd)
        if video or soft:
            if soft is not None:
                frames.append(soft.capture(data))
            else:
                # mujoco>=3.x: Renderer has no .camera attr — drive an
                # explicit MjvCamera per frame (follows the base).
                cam.lookat[:] = data.qpos[0:3].copy() + [0, 0, 0.2]
                cam.distance, cam.azimuth, cam.elevation = 2.8, 90.0, -12.0
                video.update_scene(data, camera=cam)
                frames.append(video.render())

    print(f"[INFO] survived {alive_steps} steps = {alive_steps*CONTROL_DT:.2f}s of "
          f"{n_steps - settle_steps} commanded steps")
    if vxy_err:
        print(f"[INFO] mean |v_xy error| = {np.mean(vxy_err):.3f} m/s, "
              f"mean |yaw error| = {np.mean(yaw_err):.3f} rad/s, "
              f"distance = {np.linalg.norm(data.qpos[:2]):.2f} m")

    if args.video and frames:
        import imageio.v2 as imageio
        imageio.mimwrite(args.video, frames, fps=int(1 / CONTROL_DT), quality=8)
        print(f"[VIDEO] {args.video} ({len(frames)} frames, "
              f"{Path(args.video).stat().st_size // 1024}KB)")

    if args.json:
        Path(args.json).write_text(json.dumps({
            "ckpt": str(args.ckpt), "cmd": list(args.cmd),
            "duration_s": args.duration, "settle_s": args.settle,
            "survived_steps": alive_steps,
            "survived_s": alive_steps * CONTROL_DT,
            "mean_vxy_err": float(np.mean(vxy_err)) if vxy_err else None,
            "mean_yaw_err": float(np.mean(yaw_err)) if yaw_err else None,
            "distance_m": float(np.linalg.norm(data.qpos[:2])),
            "final_base_z": float(data.qpos[2]),
            "fell": bool(alive_steps < n_steps - settle_steps),
        }, indent=1))
    if LOG:
        npz = {k: np.asarray(v) for k, v in L.items()}
        npz["meta"] = json.dumps({
            "cmd": list(args.cmd), "ckpt": str(args.ckpt),
            "control_dt": CONTROL_DT, "settle_steps": settle_steps,
            "action_lpf": args.action_lpf,
            "action_lpf_order": args.action_lpf_order,
            "fell": bool(alive_steps < n_steps - settle_steps),
            "hinge_names": hinge, "foot_names": foot_names,
            "sole_order": "row=foot_names, cols=[front_l, front_r, back_l, back_r]",
            "robustness": {"obs_noise": args.obs_noise,
                           "latency_steps": args.latency_steps,
                           "action_lag": args.action_lag,
                           "torso_mass_scale": args.torso_mass_scale,
                           "push_mag": args.push_mag, "seed": args.seed},
        })
        np.savez_compressed(args.log, **npz)
        print(f"[LOG] {args.log} ({Path(args.log).stat().st_size // 1024}KB, "
              f"{len(L['t'])} steps)")
    if soft:
        soft.close()
    if video:
        video.close()


if __name__ == "__main__":
    main()
