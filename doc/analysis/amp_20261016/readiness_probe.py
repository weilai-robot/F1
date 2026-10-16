"""Read-only deployment audit probes, 2026-10-16.

Requires numpy, mujoco 3.12.0, onnxruntime and sibling X1_29_AMP repository.
Example from F1 root:
    python doc/analysis/amp_20261016/readiness_probe.py raw_cpp clamped
    python doc/analysis/amp_20261016/readiness_probe.py identified_cpp clamped
    python doc/analysis/amp_20261016/readiness_probe.py identified clamped
    python doc/analysis/amp_20261016/readiness_probe.py identified_anklecap clamped

This adapts the existing Python harness in memory; it does not execute C++ or
ROS2, send hardware commands, or modify deployment files. The *_cpp modes
approximate the C++ torque limits: software cap only on ankles, XML actuator
caps on all joints. They do not test scheduling or sensor callback timing.
Outputs are written to /tmp/f1_amp_<mode>_clamped_20261016.json.
"""
import importlib.util
import sys
import re
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
TRAIN = ROOT.parent / 'X1_29_AMP'

def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value

mode = sys.argv[1]
clamped = len(sys.argv) > 2 and sys.argv[2] == 'clamped'
hpath = ROOT / 'scripts/amp_sim_bisect/rollout_bisect.py'
source = hpath.read_text()
if clamped:
    target_hook = '        if servo_idx is not None:\n'
    assert source.count(target_hook) == 1
    source = source.replace(target_hook, '        q_tgt_full = np.clip(q_tgt_full, [AUDIT_LIMITS[n][0] for n in hinge], [AUDIT_LIMITS[n][1] for n in hinge])\n' + target_hook)
    policy_hook = '            act = policy_fn(layers, mean, std, ob_eff)'
    assert source.count(policy_hook) == 1
    source = source.replace(policy_hook, '            act = np.clip(policy_fn(layers, mean, std, np.clip(ob_eff, -100, 100)), -100, 100)')
h = types.ModuleType('f1_harness')
h.__file__ = str(hpath)
exec(compile(source, str(hpath), 'exec'), h.__dict__)
if clamped:
    config = (ROOT / 'motion_control/module/control_module/cfg/rl_x1_sim.yaml').read_text()
    config = re.sub(r'(?m)^\s*#.*\n', '', config)
    h.AUDIT_LIMITS = {n: (float(lo), float(hi)) for n, lo, hi in re.findall(r'^  (\w+_joint):\s*\n\s+lower:\s*([-\d.]+)\s*\n\s+upper:\s*([-\d.]+)', config, re.M)}
    assert len(h.AUDIT_LIMITS) == 29
    h.FALL_TILT = 45.0
    print('[AUDIT] YAML target limits, obs/action clipping and 45deg tilt guard enabled')
c = module('training_tables', TRAIN / 'sim2sim/mujoco_rollout.py')
if mode != 'legacy_harness':
    h.PD = c.PD.copy()
    if mode.endswith('_cpp'):
        h.PD = {n: (kp, kd, 80.0 if 'ankle_' in n else float('inf')) for n, (kp, kd, _) in c.PD.items()}
    original = h.build_model
    def build(*args, **kwargs):
        import mujoco
        model, feet = original(*args, **kwargs)
        for joint, armature in c.ARMATURE.items():
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
            model.dof_armature[model.jnt_dofadr[jid]] = armature if mode.startswith('identified') else 0.0
        if mode in ('identified_traincap', 'identified_anklecap'):
            for a in range(model.nu):
                n = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, int(model.actuator_trnid[a, 0]))
                if mode == 'identified_anklecap' and 'ankle_' not in n:
                    continue
                cap = c.PD[n][2]
                model.actuator_ctrlrange[a] = [-cap, cap]
        print('[AUDIT] PD=v68 armature=' + mode)
        return model, feet
    h.build_model = build
sys.argv = ['rollout_bisect.py', '--ckpt', str(ROOT / 'motion_control/module/control_module/policy/x1_amp_v70b_m16000_50hz.onnx'),
            '--repo-root', str(ROOT), '--robot', 'deploy_flat', '--no-align',
            '--spawn-pose', 'stand', '--settle', '0', '--cmd', '1', '0', '0',
            '--duration', '12', '--pd-every', '1', '--via-actuator',
            '--json', f'/tmp/f1_amp_{mode}_{"clamped_" if clamped else ""}20261016.json']
h.main()
