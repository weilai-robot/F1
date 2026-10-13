#!/usr/bin/env python3
"""Read AMP logs without changing sources; separate fresh feedback from stop tails.

Requires numpy and pandas. scipy adds knee-cycle metrics. Plotting is separate.
Joint tracking is a command-to-feedback diagnostic, not a measurement of stride
length or world velocity. Exact cache repetition is reported, not silently used.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def config(path):
    text = Path(path).read_text()
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        try:
            import yaml
            return yaml.safe_load(text)
        except ImportError:
            return json.loads(subprocess.check_output(
                ["ruby", "-ryaml", "-rjson", "-e", "puts JSON.generate(YAML.load(STDIN.read))"],
                input=text, text=True))


def stats(x):
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    if not len(x): return None
    return {"n": len(x), "mean": float(x.mean()), "std": float(x.std()),
            "min": float(x.min()), "max": float(x.max()),
            "p05": float(np.quantile(x, .05)), "p50": float(np.quantile(x, .5)),
            "p95": float(np.quantile(x, .95)), "p99": float(np.quantile(x, .99))}


def freeze(d, columns, t):
    a = d[columns].to_numpy(dtype=float)
    unchanged = np.all((a[1:] == a[:-1]) | (np.isnan(a[1:]) & np.isnan(a[:-1])), axis=1)
    changed = np.flatnonzero(~unchanged) + 1
    last = int(changed[-1]) if len(changed) else 0
    return {"last_change_row": last, "last_change_s": float(t[last]),
            "last_change_timestamp_ns": int(d.timestamp_ns.iloc[last]),
            "constant_tail_s": float(t[-1] - t[last]),
            "unchanged_interval_fraction": float(unchanged.mean())}


def quat_matrices(q):
    q = np.asarray(q, dtype=float)
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    w, x, y, z = q.T
    return np.stack([1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y),
                     2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x),
                     2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)], axis=1).reshape(-1, 3, 3)


def pitch(r):
    return np.arctan2(-r[:, 2, 0], np.hypot(r[:, 0, 0], r[:, 1, 0]))


def chest_pitch(qbase, qyaw, qroll, qpitch):
    # Anatomical waist axes at neutral: yaw +Z, roll -X, pitch +Y.
    out = []
    for rb, y, r, p in zip(quat_matrices(qbase), qyaw, qroll, qpitch):
        cy, sy, cr, sr, cp, sp = np.cos(y), np.sin(y), np.cos(r), np.sin(r), np.cos(p), np.sin(p)
        rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
        rx = np.array([[1,0,0],[0,cr,sr],[0,-sr,cr]])
        ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
        out.append(rb @ rz @ rx @ ry)
    return pitch(np.asarray(out))


def cycle_metrics(t, q):
    try:
        from scipy.signal import find_peaks
    except ImportError:
        return {"unavailable": "scipy not installed"}
    dt = float(np.median(np.diff(t)))
    # Three-sample offline smoothing; 0.3s spacing, 0.12rad prominence.
    smooth = np.convolve(q, np.ones(3)/3, mode="same")
    peaks, _ = find_peaks(smooth, distance=max(1, round(.3/dt)), prominence=.12)
    peaks = peaks[(peaks > 1) & (peaks < len(q)-2)]
    intervals = np.diff(t[peaks])
    excursion = [float(np.max(q[a:b+1]) - np.min(q[a:b+1])) for a,b in zip(peaks,peaks[1:])]
    return {"peak_times_s": t[peaks].tolist(), "intervals_s": intervals.tolist(),
            "interval_stats": stats(intervals), "excursions_rad": excursion,
            "excursion_stats": stats(excursion),
            "period_cv": float(intervals.std()/intervals.mean()) if len(intervals)>1 else None}


def replay(d, ctl, policy):
    names = ctl["joint_list"]
    default = np.asarray(ctl["init_state"], dtype=float)
    a = d[["action_"+n for n in names]].to_numpy(dtype=np.float32)
    q = d[["pos_"+n for n in names]].to_numpy(dtype=float)
    dq = d[["vel_"+n for n in names]].to_numpy(dtype=float)
    rb = quat_matrices(d[["imu_quat_"+x for x in "wxyz"]].to_numpy())
    gravity = -rb[:, 2, :]
    previous = np.vstack([np.zeros((1,29)),a[:-1]])
    frame = np.column_stack([d[["base_ang_vel_"+x for x in "xyz"]],gravity,
        d[["cmd_linear_x","cmd_linear_y","cmd_angular_z"]],q-default,dq,previous]).astype(np.float32)
    history = np.zeros((len(d),3,96),dtype=np.float32)
    for lag in range(3):
        if lag == 0: history[:,2] = frame
        else: history[lag:,2-lag] = frame[:-lag]
    obs = np.concatenate([history[:,:,lo:hi].reshape(len(d),-1)
                          for lo,hi in [(0,3),(3,6),(6,9),(9,38),(38,67),(67,96)]],axis=1)
    obs = np.clip(obs,-100,100)
    with np.load(policy,allow_pickle=False) as z:
        x = (obs-z["mean"].reshape(-1))/z["std"].reshape(-1)
        count = sum(k.endswith("_w") for k in z.files)
        for i in range(count):
            x = x @ z[f"l{i}_w"].T + z[f"l{i}_b"]
            if i < count-1: x = np.where(x<0,np.expm1(np.minimum(x,20)),x)
    difference = np.abs(x-a)
    return {"policy":str(Path(policy).resolve()), "policy_sha256":sha256(policy),
            "mae":float(difference.mean()),
            "p99_abs_error":float(np.quantile(difference,.99)), "max_abs_error":float(difference.max()),
            "first_frame_max_error":float(difference[0].max()),
            "per_joint_mae":dict(zip(names,map(float,difference.mean(axis=0))))}


def analyze(path, cfg, start_s, policy=None):
    d = pd.read_csv(path)
    t = (d.timestamp_ns.to_numpy(dtype=np.int64)-int(d.timestamp_ns.iloc[0]))/1e9
    names = [c[4:] for c in d if c.startswith("pos_") and not c.startswith("pos_des_")]
    groups = {"all_feedback":[f"{field}_{n}" for n in names for field in ("pos","vel","effort")],
              "imu":[c for c in d if c.startswith("imu_")],
              "left_arm":[f"{field}_{n}" for n in names if n.startswith("left_") and any(x in n for x in ("shoulder","elbow","wrist")) for field in ("pos","vel","effort")],
              "right_arm":[f"{field}_{n}" for n in names if n.startswith("right_") and any(x in n for x in ("shoulder","elbow","wrist")) for field in ("pos","vel","effort")]}
    freezes = {key:freeze(d,columns,t) for key,columns in groups.items()}
    end_s = freezes["all_feedback"]["last_change_s"]
    valid = (t >= start_s) & (t <= end_s)
    if valid.sum() < 10: raise ValueError("fresh-data window too short")
    dt = float(np.median(np.diff(t[valid])))
    ctl = cfg["controllers"]["rl_walk_amp"]
    pmap = dict(zip(ctl["joint_list"],ctl["stiffness"]))
    dmap = dict(zip(ctl["joint_list"],ctl["damping"]))
    joints, reconstruction, target_series = {}, {}, {}
    for name in names:
        q, dq, raw = (d[k+name].to_numpy(dtype=float) for k in ("pos_","vel_","pos_des_raw_"))
        if np.isfinite(d["pos_des_lpf_"+name]).all():
            target = d["pos_des_lpf_"+name].to_numpy(dtype=float)
        else:
            # Ankle post-limit target is absent. Reconstruct the target used by
            # software PD; cross-check against configuration clipping below.
            target = q + (d["tau_des_raw_"+name].to_numpy()+dmap[name]*dq)/pmap[name]
        bound = cfg["joint_limits"][name]
        reconstruction[name] = float(np.max(np.abs(target-np.clip(raw,bound["lower"],bound["upper"]))))
        target_series[name] = target
        qs, ds, e = q[valid], target[valid], (target-q)[valid]
        qstats, dstats = stats(qs), stats(ds)
        best = None
        if qstats["std"] > 1e-6 and dstats["std"] > 1e-6:
            for lag in range(round(.14/dt)+1):
                xx, yy = (ds[:-lag],qs[lag:]) if lag else (ds,qs)
                corr = float(np.corrcoef(xx,yy)[0,1])
                if np.isfinite(corr) and (best is None or corr > best["correlation"]):
                    best = {"lag_ms":1000*dt*lag,"correlation":corr,"mae_rad":float(np.mean(abs(xx-yy)))}
        derivative = np.gradient(qs,t[valid])
        measured_velocity = dq[valid]
        if np.std(measured_velocity)>1e-6 and np.std(derivative)>1e-6:
            slope = float(np.cov(measured_velocity,derivative,bias=True)[0,1]/np.var(measured_velocity))
            velocity_check = {"position_derivative_vs_velocity_slope":slope,
                              "correlation":float(np.corrcoef(measured_velocity,derivative)[0,1])}
        else:
            velocity_check = {"position_derivative_range_rad_s":float(np.ptp(derivative)),
                              "reported_velocity_mean_rad_s":float(np.mean(measured_velocity))}
        joints[name] = {"q":qstats,"target":dstats,"mae_rad":float(np.mean(abs(e))),
            "p95_abs_error_rad":float(np.quantile(abs(e),.95)),"bias_rad":float(e.mean()),
            "q_rom_deg":float(np.degrees(qstats["p95"]-qstats["p05"])),
            "target_rom_deg":float(np.degrees(dstats["p95"]-dstats["p05"])),
            "target_clipped_fraction":float(np.mean(abs(target[valid]-raw[valid])>1e-5)),
            "best_positive_lag":best,"velocity_consistency":velocity_check,
            "freeze":freeze(d,[f"{f}_{name}" for f in ("pos","vel","effort")],t)}
    qb = d[["imu_quat_"+x for x in "wxyz"]].to_numpy()
    rb = quat_matrices(qb)
    chest = chest_pitch(qb,*(d["pos_lumbar_"+a+"_joint"].to_numpy() for a in ("yaw","roll","pitch")))
    heading = np.unwrap(np.arctan2(rb[:,1,0],rb[:,0,0]))
    active = t <= end_s
    motors = {}
    for col in d:
        if col.startswith("actuator_cmd_effort_"):
            motor = col.removeprefix("actuator_cmd_effort_")
            x = d[col].to_numpy()[valid]
            motors[motor] = {"command":stats(x),"command_over_50_fraction":float(np.mean(abs(x)>50)),
                            "feedback":stats(d["actuator_state_effort_"+motor].to_numpy()[valid])}
    cycles = {side:cycle_metrics(t[valid],d["pos_"+side+"_knee_pitch_joint"].to_numpy()[valid]) for side in ("left","right")}
    target_cycles = {side:cycle_metrics(t[valid],target_series[side+"_knee_pitch_joint"][valid]) for side in ("left","right")}
    correlations = {}
    for kind in ("hip_pitch","knee_pitch","hip_yaw","shoulder_pitch"):
        left,right = "left_"+kind+"_joint","right_"+kind+"_joint"
        for field in ("feedback","target"):
            a,b = (d["pos_"+left].to_numpy()[valid],d["pos_"+right].to_numpy()[valid]) if field=="feedback" else (target_series[left][valid],target_series[right][valid])
            correlations[kind+"_"+field] = float(np.corrcoef(a,b)[0,1]) if min(np.std(a),np.std(b))>1e-6 else None
    data = {"source":str(Path(path).resolve()),"sha256":sha256(path),
            "rows":len(d),"start_timestamp_ns":int(d.timestamp_ns.iloc[0]),"duration_s":float(t[-1]),
            "sample_period_ms":stats(np.diff(t)*1000),"analysis_window_s":[start_s,end_s],
            "active_rows":int(active.sum()),"analysis_rows":int(valid.sum()),
            "command":{key:stats(d[key]) for key in ("cmd_linear_x","cmd_linear_y","cmd_angular_z")},
            "freeze_groups":freezes,"joints":joints,"motors":motors,"cycles":cycles,
            "target_cycles":target_cycles,"bilateral_correlations":correlations,
            "ankle_target_reconstruction_assumes_current_nominal_pd":True,
            "target_reconstruction_max_error_rad":reconstruction,
            "base_pitch_deg":stats(np.degrees(pitch(rb)[valid])),
            "chest_pitch_model_estimate_deg":stats(np.degrees(chest[valid])),
            "heading_change_active_deg":float(np.degrees(heading[active][-1]-heading[0])),
            "gyro_z_integral_active_deg":float(np.degrees(np.sum(
                (d.base_ang_vel_z.to_numpy()[active][:-1]+d.base_ang_vel_z.to_numpy()[active][1:])
                *np.diff(t[active])/2))),
            "gyro_z_active_rad_s":stats(d.base_ang_vel_z.to_numpy()[active]),
            "imu_coordinate_calibration_required_for_chest_estimate":True}
    if policy: data["policy_replay"] = replay(d,ctl,policy)
    return data


def simulation(path, start_s):
    with np.load(path,allow_pickle=False) as z:
        meta = json.loads(str(z["meta"]))
        t,q,qb = z["t"],z["q"],z["base_quat"]
        names = meta["hinge_names"]
        valid = t >= start_s
        joints = {}
        for i,name in enumerate(names):
            s = stats(q[valid,i]);joints[name] = {"q":s,"q_rom_deg":float(np.degrees(s["p95"]-s["p05"]))}
        w = [q[:,names.index("lumbar_"+a+"_joint")] for a in ("yaw","roll","pitch")]
        rb=quat_matrices(qb);chest=chest_pitch(qb,*w)
        return {"source":str(Path(path).resolve()),"sha256":sha256(path),
            "meta":meta,"analysis_window_s":[start_s,float(t[-1])],
            "joints":joints,"base_pitch_deg":stats(np.degrees(pitch(rb)[valid])),
            "chest_pitch_model_estimate_deg":stats(np.degrees(chest[valid])),
            "forward_velocity_m_s":stats(z["v_b"][valid,0]),
            "cycles":{side:cycle_metrics(t[valid],q[valid,names.index(side+"_knee_pitch_joint")]) for side in ("left","right")},
            "shoulder_pitch_antiphase_correlation":float(np.corrcoef(q[valid,names.index("left_shoulder_pitch_joint")],q[valid,names.index("right_shoulder_pitch_joint")])[0,1])}


def ethercat_stats(path, runs):
    rows = pd.read_csv(path).to_dict(orient="records")
    windows = [(r["start_timestamp_ns"],
                r["start_timestamp_ns"]+int(r["analysis_window_s"][1]*1e9)) for r in runs]
    for row in rows:
        row["cycle_late_avg_ms"] = row["cycle_late_avg_ns"]/1e6
        row["cycle_late_max_ms"] = row["cycle_late_max_ns"]/1e6
        row["overrun_fraction"] = row["cycle_overruns"]/row["cycle_cnt"] if row["cycle_cnt"] else None
        row["dc_min_invalid_sentinel"] = row["dc_dev_abs_min_ns"] == -(2**63)
        stamp = pd.Timestamp(pd.to_datetime(str(row["wall_ts"]),format="%Y%m%d_%H%M%S"),tz="Asia/Shanghai").value
        row["timestamp_within_fresh_walking"] = any(a <= stamp <= b for a,b in windows)
    return {"source":str(Path(path).resolve()), "sha256":sha256(path), "rows":rows,
            "wkc_bad_polls_unit":"100ms error-handler polls, not dropped frames",
            "overrun_definition":"wake lateness > configured EtherCAT cycle / 5",
            "current_source_stats_defects":[
                "absolute-min initialized/reset to INT64_MIN: positive samples cannot lower it",
                "first DC delta subtracts zero instead of a previous DC clock sample"]}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("logs",type=Path,nargs="+")
    p.add_argument("--config",type=Path,default=ROOT/"motion_control/module/control_module/cfg/rl_x1.yaml")
    p.add_argument("--steady-start",type=float,default=1.0)
    p.add_argument("--sim",type=Path)
    p.add_argument("--policy",type=Path)
    p.add_argument("--ecat",type=Path)
    p.add_argument("--out",type=Path,required=True)
    args=p.parse_args()
    cfg=config(args.config)
    result={"steady_start_s":args.steady_start,"config_source":str(args.config.resolve()),
            "config_sha256":sha256(args.config),
            "runs":[analyze(path,cfg,args.steady_start,args.policy) for path in args.logs]}
    if args.sim:result["simulation"]=simulation(args.sim,max(1.5,args.steady_start))
    if args.ecat:result["ethercat_stats"]=ethercat_stats(args.ecat,result["runs"])
    args.out.mkdir(parents=True,exist_ok=True)
    (args.out/"metrics.json").write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+"\n")
    rows=[]
    for run in result["runs"]:
        for name,j in run["joints"].items():
            rows.append({"run":Path(run["source"]).stem,"joint":name,"window_start_s":run["analysis_window_s"][0],
                "window_end_s":run["analysis_window_s"][1],"mae_deg":np.degrees(j["mae_rad"]),
                "bias_deg":np.degrees(j["bias_rad"]),"q_rom_deg":j["q_rom_deg"],"target_rom_deg":j["target_rom_deg"],
                "target_clipped_fraction":j["target_clipped_fraction"],"feedback_last_change_s":j["freeze"]["last_change_s"]})
    pd.DataFrame(rows).to_csv(args.out/"joint_metrics.csv",index=False)
    for run in result["runs"]:
        print(Path(run["source"]).name,"fresh window",run["analysis_window_s"],"policy replay",run.get("policy_replay",{}).get("max_abs_error"))


if __name__=="__main__": main()
