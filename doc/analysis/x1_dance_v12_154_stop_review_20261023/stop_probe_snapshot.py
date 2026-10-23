from pathlib import Path
import argparse, ast, hashlib, json, sys
import torch
CONTROL = Path('/Users/yumx/code/F1/motion_control/module/control_module')
SOURCE = CONTROL/'tools/sim_beyondmimic_x1.py'
sys.path.insert(0, str(SOURCE.parent))
postroll = '''
# Audit-only extension: same model, final physical state, and YAML pd_keep gains.
# Production PDController snapshots measured q at restart and uses qd_des=0.
# All actuators remain ideal effort-saturated joint PD, as in the original probe.
if fault is None and frame_count == len(ref_q):
    mujoco.mj_forward(model, data)
    keep_cfg = cfg['controllers']['pd_keep']
    keep_target = data.qpos[qadr].copy()
    keep_kp = np.zeros(29)
    keep_kd = np.zeros(29)
    for name, p, d in zip(keep_cfg['joint_list'], keep_cfg['stiffness'], keep_cfg['damping']):
        i = names.index(name)
        keep_kp[i], keep_kd[i] = p, d
    qd_at_switch = data.qvel[vadr].copy()
    last_policy_torque = np.clip(kp*(target-keep_target)-kd*qd_at_switch, -effort, effort)
    keep_initial_torque = np.clip(-keep_kd*qd_at_switch, -effort, effort)
    def tilt_now():
        return float(np.degrees(np.arccos(np.clip(math.quat_to_rotmat(data.qpos[3:7])[2, 2], -1, 1))))
    margin, both_feet = com_margin()
    postroll = dict(
        method='Audit extension of production Python probe; captures final measured q, immediately switches to YAML pd_keep gains, qd_des=0. Ideal effort-saturated joint PD; no ROS/DCU or actuator-level implementation.',
        requested_hold_s=8.0,
        transition_s=0.0,
        base_z_at_switch_m=float(data.qpos[2]),
        base_tilt_at_switch_deg=tilt_now(),
        base_linear_speed_at_switch_m_s=float(np.linalg.norm(data.qvel[:3])),
        base_gyro_at_switch_rad_s=data.qvel[3:6].tolist(),
        base_angular_speed_at_switch_rad_s=float(np.linalg.norm(data.qvel[3:6])),
        max_joint_speed_at_switch_rad_s=float(np.max(np.abs(qd_at_switch))),
        com_support_margin_at_switch_m=margin,
        both_feet_touching_at_switch=both_feet,
        max_initial_joint_torque_step_Nm=float(np.max(np.abs(last_policy_torque-keep_initial_torque))),
        first_tilt_above_35_deg_s=None,
        first_tilt_above_60_deg_s=None,
        first_base_z_below_0p35_m_s=None,
        max_tilt_deg=tilt_now(), min_base_z_m=float(data.qpos[2]),
        max_hold_pose_error_rad=0.0, samples=[])
    for post_tick in range(8001):
        mujoco.mj_forward(model, data)
        elapsed = post_tick/1000.0
        tilt = tilt_now()
        z = float(data.qpos[2])
        postroll['max_tilt_deg'] = max(postroll['max_tilt_deg'], tilt)
        postroll['min_base_z_m'] = min(postroll['min_base_z_m'], z)
        postroll['max_hold_pose_error_rad'] = max(postroll['max_hold_pose_error_rad'], float(np.max(np.abs(data.qpos[qadr]-keep_target))))
        for key, violation in [('first_tilt_above_35_deg_s', tilt > 35), ('first_tilt_above_60_deg_s', tilt > 60), ('first_base_z_below_0p35_m_s', z < .35)]:
            if violation and postroll[key] is None:
                postroll[key] = elapsed
        if post_tick % 500 == 0:
            postroll['samples'].append(dict(t_s=elapsed, tilt_deg=tilt, base_z_m=z,
                base_linear_speed_m_s=float(np.linalg.norm(data.qvel[:3])),
                base_angular_speed_rad_s=float(np.linalg.norm(data.qvel[3:6])),
                max_joint_speed_rad_s=float(np.max(np.abs(data.qvel[vadr])))))
        if post_tick == 8000:
            break
        q, qd = data.qpos[qadr], data.qvel[vadr]
        data.ctrl[actuators] = np.clip(keep_kp*(keep_target-q)-keep_kd*qd, -effort, effort)
        mujoco.mj_step(model, data)
    result['stop_audit'] = postroll
'''

def make_run():
    tree = ast.parse(SOURCE.read_text(), filename=str(SOURCE))
    run = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'run')
    # Append the audit immediately before the existing return finish(result),
    # preserving all original playback statistics and original simulation logic.
    assert isinstance(run.body[-1], ast.Return)
    run.body[-1:-1] = ast.parse(postroll).body
    ast.fix_missing_locations(tree)
    namespace = {'__file__':str(SOURCE),'__name__':'x1_stop_review'}
    exec(compile(tree,str(SOURCE),'exec'), namespace)
    return namespace['run']

p=argparse.ArgumentParser()
p.add_argument('--yaw',type=float,default=0)
p.add_argument('--out',type=Path,required=True)
a=p.parse_args()
torch.set_num_threads(1)
result=make_run()(Path('/Users/yumx/code/Beyondminic-Weilai-X1'),3.0,a.yaw,10.0)
result['audit_source_sha256']=hashlib.sha256(SOURCE.read_bytes()).hexdigest()
result['audit_script_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
a.out.parent.mkdir(parents=True,exist_ok=True)
a.out.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result['stop_audit'].items() if k != 'samples'},indent=2))
