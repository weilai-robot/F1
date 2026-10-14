#!/usr/bin/env python3
"""Prepare read-only DCU instrumentation; analyze feedback freezes and leg response.

prepare writes separate runtime configs; it sends no robot messages. analyze
preserves raw inputs. Requires PyYAML for prepare; NumPy/pandas for analysis.
"""
import argparse
import copy
import hashlib
import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def load_config(path):
    text=Path(path).read_text()
    try:return json.loads(text)
    except json.JSONDecodeError:
        try:
            import yaml
            return yaml.safe_load(text)
        except ImportError:
            return json.loads(subprocess.check_output(
                ["ruby","-ryaml","-rjson","-e","puts JSON.generate(YAML.load(STDIN.read))"],
                input=text,text=True))


def prepare(args):
    import upper_direction_test as upper
    folder = args.bin_dir.resolve()
    source = folder / (args.base_config or (
        "cfg/x1_cfg_real_nav.yaml" if args.scenario == "static" else "cfg/x1_cfg.yaml"))
    main = load_config(source)
    modules = {m["name"]: m for m in main["aimrt"]["module"]["modules"]}
    control_path = upper.resolve_config(folder, modules["ControlModule"]["cfg_file_path"])
    dcu_path = upper.resolve_config(folder, modules["DcuDriverModule"]["cfg_file_path"])
    control, dcu = load_config(control_path), load_config(dcu_path)
    routes = {}
    occupied = set()
    for bus in dcu["dcu_network"]:
        if not bus["enable"]: continue
        for channel in range(1,4):
            for motor in bus.get(f"channel_{channel}",[]):
                slot = (bus["ecat_id"], channel, motor["can_id"])
                if not 1 <= motor["can_id"] <= 8 or slot in occupied or motor["name"] in routes:
                    raise ValueError(f"invalid/duplicate motor route: {motor}")
                occupied.add(slot)
                routes[motor["name"]] = {"dcu":bus["name"],"ecat_id":slot[0],"channel":slot[1],"can_id":slot[2],"type":motor["type"]}
    missing = set(dcu["actuator_list"])-routes.keys()
    if missing: raise ValueError(f"unregistered actuators: {sorted(missing)}")
    active_modules = [name for pkg in main["aimrt"]["module"]["pkgs"] for name in pkg.get("enable_modules",[])]
    if "SimModule" in active_modules: raise ValueError("use a real-robot config without SimModule")
    sources = {str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (source,control_path,dcu_path)}
    plan = None
    if args.scenario == "static":
        control, main, plan = upper.build_configs(control,main,dcu,args.angle)
        plan["bin_dir"] = str(folder)
        plan["sources"] = sources
        upper.print_cases(plan)
        main_rel, control_rel = upper.MAIN_REL, upper.CONTROL_REL
    else:
        if "rl_walk_amp" not in control["controllers"]: raise ValueError("AMP controller missing")
        main_rel, control_rel = Path("cfg/x1_cfg_amp_priority.yaml"), None
    dcu_rel = Path("cfg/dcu_driver_module/dcu_priority.yaml")
    dcu = copy.deepcopy(dcu)
    dcu["diagnostics_conf"] = {"enabled":True,"frequency_hz":50,"duration_s":args.duration,"output_dir":"test_logs/dcu_diag"}
    dcu["actuator_debug"] = True
    for mod in main["aimrt"]["module"]["modules"]:
        if mod["name"] == "DcuDriverModule":
            mod["cfg_file_path"] = "./"+str(dcu_rel)
            mod["log_lvl"] = "INFO"
    manifest = {"scenario":args.scenario,"sources":sources,"main_config":str(main_rel),
        "routes":routes,"joint_offset":control.get("joint_offset",{}),"transmission":dcu["transmission"],
        "control_config":str(control_path),"diagnostics_conf":dcu["diagnostics_conf"],
        "amp_gain_direction_policy_changes":False}
    if args.dry_run:
        print(json.dumps(manifest,ensure_ascii=False,indent=2));return
    encode = lambda obj: json.dumps(obj,ensure_ascii=False,indent=2)+"\n"
    (folder/dcu_rel).parent.mkdir(parents=True,exist_ok=True)
    (folder/dcu_rel).write_text(encode(dcu))
    (folder/main_rel).write_text(encode(main))
    if plan is not None:
        encoded = encode(control)
        (folder/control_rel).write_text(encoded)
        plan["control_sha256"] = hashlib.sha256(encoded.encode()).hexdigest()
        plan["main_sha256"] = hashlib.sha256(encode(main).encode()).hexdigest()
        (folder/upper.MANIFEST_REL).write_text(encode(plan))
    (folder/"cfg/amp_priority_manifest.json").write_text(encode(manifest))
    print("Generated",folder/main_rel)
    print("DCU CSV:",folder/"test_logs/dcu_diag",f"(50 Hz, {args.duration:g} s; AMP gains/direction unchanged)")


def summarize_dcu(path, bounds=None):
    import numpy as np
    import pandas as pd
    d = pd.read_csv(path,dtype={"rx_hex":str,"tx_hex":str})
    if bounds:
        d=d[(d.timestamp_ns>=bounds[0])&(d.timestamp_ns<=bounds[1])]
    if d.empty: raise ValueError("DCU log has no samples in selected window")
    out = {"source":str(path.resolve()),"sha256":hashlib.sha256(path.read_bytes()).hexdigest(),
           "window_timestamp_ns":[int(d.timestamp_ns.min()),int(d.timestamp_ns.max())],"motors":{},"groups":[]}
    for name,g in d.groupby("actuator",sort=False):
        r = g.tx_format.eq(1) & g.rx_seen.eq(1)
        # A stationary motor can repeat bytes indefinitely. This flag requires
        # long unchanged measurement bytes AND a claimed nonzero velocity.
        contradiction = (g.rx_measurement_age_ms>=200)&(g.velocity.abs()>=.1)&g.rx_seen.eq(1)
        link_late = (g.rx_age_ms>100)|g.rx_seen.eq(0)
        def ROM(x):
            x=x[np.isfinite(x)]
            return float(np.degrees(x.quantile(.95)-x.quantile(.05))) if len(x) else None
        item={"rows":len(g),"route":[int(g.ecat_id.iloc[0]),int(g.channel.iloc[0]),int(g.can_id.iloc[0])],
              "rx_age_max_ms":float(g.rx_age_ms.max()),"rx_content_age_max_ms":float(g.rx_content_age_ms.max()),
              "rx_measurement_age_max_ms":float(g.rx_measurement_age_ms.max()),
              "pdo_seq_span":int(g.rx_pdo_seq.max()-g.rx_pdo_seq.min()),
              "measurement_seq_span":int(g.rx_measurement_seq.max()-g.rx_measurement_seq.min()),
              "moving_feedback_repetition_fraction":float(contradiction.mean()),
              "pdo_late_fraction":float(link_late.mean()),
              "slot_disabled_fraction":float(g.tx_slot_enabled.eq(0).mean()),
              "status_error_nibble_nonzero_fraction":float((g.error_nibble>0).mean()),
              "q_rom_deg":ROM(g.position),"target_rom_deg":ROM(g.loc[g.tx_format.isin([1,2]),"cmd_position"]),
              "host_decode_gain_ranges_are_not_firmware_readback":True}
        first=g.loc[contradiction,"timestamp_ns"]
        item["first_repetition_timestamp_ns"]=int(first.iloc[0]) if len(first) else None
        if r.any():
            v=g.loc[r]
            item.update({"encoded_kp_min_max":[float(v.cmd_kp.min()),float(v.cmd_kp.max())],
                "encoded_kd_min_max":[float(v.cmd_kd.min()),float(v.cmd_kd.max())],
                "feedback_near_host_effort_endpoint_fraction":float((v.effort.abs()>=.98*v.host_effort_max).mean()),
                "cmd_cache_position_p95_error_rad":float((v.cmd_position-v.cache_cmd_position).abs().quantile(.95)),
                "cmd_cache_kp_p95_error":float((v.cmd_kp-v.cache_cmd_kp).abs().quantile(.95)),
                "cmd_cache_kd_p95_error":float((v.cmd_kd-v.cache_cmd_kd).abs().quantile(.95)),
                "host_pd_effort_abs_p95":float(v.host_pd_effort.abs().quantile(.95)),
                "joint_cmd_age_max_ms":float(v.joint_cmd_age_ms.max())})
        out["motors"][name]=item
    for (ecat,ch),g in d.groupby(["ecat_id","channel"]):
        suspect=g[(g.rx_measurement_age_ms>=200)&(g.velocity.abs()>=.1)&g.rx_seen.eq(1)]
        out["groups"].append({"ecat_id":int(ecat),"channel":int(ch),
            "motors_with_moving_feedback_repetition":sorted(suspect.actuator.unique().tolist()),
            "pdo_age_max_ms":float(g.rx_age_ms.max())})
    return out


def analyze(args):
    import numpy as np
    import pandas as pd
    import analyze_amp_walk_logs as walk
    result={"notes":["payload change age is not CAN packet age; static repeated feedback is allowed",
                      "host_pd_effort assumes host-decoded gains and is not measured motor torque",
                      "post-manual-stop tails excluded when --walk-log supplied"],"runs":[]}
    cfg=walk.config(args.control_config)
    logs=args.walk_log or [None]
    for path in logs:
        run={};bounds=None
        if path:
            base=walk.analyze(path,cfg,1.0)
            run["walk_source"]=base["source"];run["walk_sha256"]=base["sha256"]
            run["window_s"]=base["analysis_window_s"];run["feedback_groups"]=base["freeze_groups"]
            bounds=[base["start_timestamp_ns"]+int(s*1e9) for s in base["analysis_window_s"]]
            run["legs"]={}
            for side in ("left","right"):
                for kind in ("hip_pitch","knee_pitch"):
                    n=side+"_"+kind+"_joint";j=base["joints"][n]
                    item={"q_rom_deg":j["q_rom_deg"],"target_rom_deg":j["target_rom_deg"],
                          "rom_ratio":j["q_rom_deg"]/j["target_rom_deg"] if j["target_rom_deg"]>1e-9 else None,
                          "apparent_lag":j["best_positive_lag"],
                          "target_clipped_fraction":j["target_clipped_fraction"]}
                    run["legs"][n]=item
            d=pd.read_csv(path);ix=(d.timestamp_ns>=bounds[0])&(d.timestamp_ns<=bounds[1])
            for side in ("left","right"):
                n=side+"_knee_pitch_joint"
                run["legs"][n]["reported_effort_near_host_100_endpoint_fraction"]=float((d.loc[ix,"effort_"+n].abs()>=98).mean())
                run["legs"][n]["actual_max_deg"]=float(np.degrees(d.loc[ix,"pos_"+n].max()))
                run["legs"][n]["largest_logged_pre_stop_target_deg"]=float(np.degrees(d.loc[d.timestamp_ns<=bounds[1],"pos_des_lpf_"+n].max()))
        if args.dcu_log: run["dcu"]=summarize_dcu(args.dcu_log,bounds)
        result["runs"].append(run)
    args.out.mkdir(parents=True,exist_ok=True)
    (args.out/"priority_summary.json").write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+"\n")
    lines=["AMP 优先排查：左臂反馈 / 髋膝响应",""]
    for run in result["runs"]:
        if "legs" in run:
            lines += [Path(run["walk_source"]).name+f" 有效窗口 {run['window_s']}"]
            for name,j in run["legs"].items():
                ratio=f"{j['rom_ratio']:.3f}" if j['rom_ratio'] is not None else "目标无摆幅"
                lines.append(f"  {name}: 目标摆幅 {j['target_rom_deg']:.2f}°，实际 {j['q_rom_deg']:.2f}°，比值 {ratio}")
        if "dcu" in run:
            for group in run["dcu"]["groups"]:
                lines.append(f"  ECAT {group['ecat_id']} / channel_{group['channel']}: 可疑运动反馈重复 {group['motors_with_moving_feedback_repetition']}")
            for name,j in run["dcu"]["motors"].items():
                if j["pdo_late_fraction"] or j["slot_disabled_fraction"] or j["status_error_nibble_nonzero_fraction"]:
                    lines.append(f"  {name}: PDO迟到 {j['pdo_late_fraction']:.1%}，发送槽未启用 {j['slot_disabled_fraction']:.1%}，状态error nibble非零 {j['status_error_nibble_nonzero_fraction']:.1%}")
        else: lines.append("  缺少新 DCU 原始诊断，无法确认逐槽反馈和编码后 kp/kd。")
    lines += ["", "重复测量与非零速度矛盾是可疑反馈证据，不等同于已定位 CAN 硬件故障。",
              "编码后 kp/kd 仍按主机量程解码；需核实电机固件量程与实际制动力矩。"]
    text="\n".join(lines)+"\n";(args.out/"priority_summary.txt").write_text(text);print(text)


def main():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest="mode",required=True)
    prep=sub.add_parser("prepare");prep.add_argument("--bin-dir",type=Path,default=ROOT/"build")
    prep.add_argument("--scenario",choices=["static","amp"],default="static")
    prep.add_argument("--base-config");prep.add_argument("--angle",type=float,default=.1)
    prep.add_argument("--duration",type=float,default=120);prep.add_argument("--dry-run",action="store_true")
    a=sub.add_parser("analyze");a.add_argument("--dcu-log",type=Path)
    a.add_argument("--walk-log",type=Path,action="append")
    a.add_argument("--control-config",type=Path,default=ROOT/"motion_control/module/control_module/cfg/rl_x1.yaml")
    a.add_argument("--out",type=Path,required=True)
    args=p.parse_args()
    if args.mode=="prepare":
        if not 0<args.duration<=600:p.error("duration must be in (0,600]")
        prepare(args)
    else:
        if not args.dcu_log and not args.walk_log:p.error("provide --dcu-log and/or --walk-log")
        analyze(args)


if __name__=="__main__":main()
