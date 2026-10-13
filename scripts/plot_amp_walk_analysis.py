#!/usr/bin/env python3
"""Scientific plots for analyze_amp_walk_logs.py; original logs stay unchanged."""
import argparse
import json
import os
from pathlib import Path
import numpy as np
import pandas as pd


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("metrics",type=Path)
    p.add_argument("--out",type=Path,required=True)
    args=p.parse_args()
    args.out.mkdir(parents=True,exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR",str(args.out/".mpl_cache"))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import analyze_amp_walk_logs as analysis
    plt.rcParams.update({"font.size":11,"axes.spines.top":False,"axes.spines.right":False,
                         "axes.grid":True,"grid.alpha":.2,"figure.dpi":160})
    metrics=json.loads(args.metrics.read_text())
    runs=metrics["runs"]
    if len(runs) != 2 or "simulation" not in metrics:
        p.error("comparison plots require exactly two walking logs and --sim during analysis")
    frames=[pd.read_csv(r["source"]) for r in runs]
    times=[(d.timestamp_ns-d.timestamp_ns.iloc[0]).to_numpy()/1e9 for d in frames]
    labels=[f"Run {i+1} (" + pd.Timestamp(r["start_timestamp_ns"],unit="ns",tz="UTC").tz_convert("Asia/Shanghai").strftime("%H:%M:%S") + ")"
            for i,r in enumerate(runs)]
    blue,orange,green="#2563EB","#D97706","#16856A"

    def save(fig,name):
        fig.savefig(args.out/name,bbox_inches="tight",facecolor="white")
        plt.close(fig)

    fig,axes=plt.subplots(3,2,figsize=(13,9),sharex="col",layout="constrained")
    for col,(r,d,t) in enumerate(zip(runs,frames,times)):
        stop=r["analysis_window_s"][1]
        left_stop=r["freeze_groups"]["left_arm"]["last_change_s"]
        axes[0,col].set_title(labels[col])
        axes[0,col].plot(t,np.degrees(d.pos_des_lpf_left_shoulder_pitch_joint),color=orange,label="Left shoulder target")
        axes[0,col].plot(t,np.degrees(d.pos_left_shoulder_pitch_joint),color=blue,label="Left shoulder feedback")
        axes[0,col].set_ylabel("Shoulder pitch (deg)")
        axes[1,col].plot(t,d.vel_left_shoulder_pitch_joint,color=orange,label="Reported velocity")
        axes[1,col].plot(t,np.gradient(d.pos_left_shoulder_pitch_joint,t),color=blue,label="Position derivative")
        axes[1,col].set_ylabel("Velocity (rad/s)")
        names=list(r["joints"])
        a=d[["pos_"+n for n in names]].to_numpy()
        changed=np.sum(a[1:]!=a[:-1],axis=1)
        axes[2,col].plot(t[1:],changed,color=green,label="Joints with changed position")
        axes[2,col].set_ylim(-1,30)
        axes[2,col].set_ylabel("Changed joints per sample")
        axes[2,col].set_xlabel("Time since AMP entry (s)")
        for ax in axes[:,col]:
            ax.axvline(left_stop,color="#B91C1C",ls=":",lw=1.5,label="Left arm cache stops" if ax is axes[0,col] else None)
            ax.axvspan(stop,t[-1],color="#CBD5E1",alpha=.55,label="After manual stop" if ax is axes[0,col] else None)
            ax.legend(loc="best",fontsize=9)
    fig.suptitle("Left-arm feedback stops before the later manual stop",fontsize=14)
    save(fig,"feedback_health.png")

    sim=metrics["simulation"]
    with np.load(sim["source"],allow_pickle=False) as z:
        st=z["t"].copy();sq=z["q"].copy();sqb=z["base_quat"].copy()
    sn=sim["meta"]["hinge_names"]
    sim_start = sim["analysis_window_s"][0]
    fig,axes=plt.subplots(2,3,figsize=(15,7),sharex="col",sharey="row",layout="constrained")
    for col in range(3):
        for row,side in enumerate(("left","right")):
            ax=axes[row,col]
            name=side+"_knee_pitch_joint"
            if col<2:
                r,d,t=runs[col],frames[col],times[col]
                start,stop=r["analysis_window_s"]
                ix=(t>=start)&(t<=stop)
                ax.step(t[ix]-start,np.degrees(d["pos_des_lpf_"+name].to_numpy()[ix]),where="post",color=orange,alpha=.8,label="Target")
                ax.plot(t[ix]-start,np.degrees(d["pos_"+name].to_numpy()[ix]),color=blue,label="Feedback")
                title=labels[col]
            else:
                ix=(st>=sim_start)&(st<=sim_start+4.8)
                ax.plot(st[ix]-sim_start,np.degrees(sq[ix,sn.index(name)]),color=blue,label="Simulation angle")
                title="Training sim2sim reference"
            if row==0:ax.set_title(title)
            ax.set_ylabel(f"{side.title()} knee pitch (deg)")
            ax.set_xlabel("Time in comparison window (s)")
            ax.legend(loc="upper right",fontsize=9)
    fig.suptitle("Knee excursions and timing: fresh real feedback versus training sim2sim",fontsize=14)
    save(fig,"knee_tracking.png")

    fig,axes=plt.subplots(2,2,figsize=(13,7),layout="constrained")
    sim_periods=np.concatenate([sim["cycles"][side]["intervals_s"] for side in ("left","right")])
    sim_excursions=np.degrees(np.concatenate([sim["cycles"][side]["excursions_rad"] for side in ("left","right")]))
    for col,r in enumerate(runs):
        axes[0,col].set_title(labels[col])
        for side,color in (("left",blue),("right",orange)):
            c=r["cycles"][side]
            periods=c["intervals_s"]
            excursions=np.degrees(c["excursions_rad"])
            axes[0,col].plot(np.arange(1,len(periods)+1),periods,"o-",color=color,label=side.title())
            axes[1,col].plot(np.arange(1,len(excursions)+1),excursions,"o-",color=color,label=side.title())
        axes[0,col].axhspan(sim_periods.min(),sim_periods.max(),color=green,alpha=.2,label=f"Sim2sim {sim_periods.min():.2f}-{sim_periods.max():.2f} s")
        axes[1,col].axhspan(sim_excursions.min(),sim_excursions.max(),color=green,alpha=.2,label="Sim2sim knee excursion")
        axes[0,col].set_ylabel("Same-leg knee cycle interval (s)")
        axes[1,col].set_ylabel("Knee peak-to-trough excursion (deg)")
        for ax in axes[:,col]:
            ax.set_xlabel("Cycle index")
            ax.legend(fontsize=9)
    fig.suptitle("Cycle variability; knee excursion is not stride length",fontsize=14)
    save(fig,"cycle_variability.png")

    fig,axes=plt.subplots(4,3,figsize=(15,12),sharex="col",sharey="row",layout="constrained")
    for col in range(3):
        if col<2:
            r,d,t=runs[col],frames[col],times[col]
            start,stop=r["analysis_window_s"]
            ix=(t>=start)&(t<=stop);x=t[ix]-start
            qb=d[["imu_quat_"+k for k in "wxyz"]].to_numpy()
            waist=d.pos_lumbar_pitch_joint.to_numpy()
            base=analysis.pitch(analysis.quat_matrices(qb))
            chest=analysis.chest_pitch(qb,*(d["pos_lumbar_"+k+"_joint"].to_numpy() for k in ("yaw","roll","pitch")))
            arms=[d["pos_"+side+"_shoulder_pitch_joint"].to_numpy() for side in ("left","right")]
            title=labels[col]
        else:
            ix=(st>=sim_start)&(st<=sim_start+4.8);x=st[ix]-sim_start
            qb=sqb;waist=sq[:,sn.index("lumbar_pitch_joint")]
            base=analysis.pitch(analysis.quat_matrices(qb))
            chest=analysis.chest_pitch(qb,*(sq[:,sn.index("lumbar_"+k+"_joint")] for k in ("yaw","roll","pitch")))
            arms=[sq[:,sn.index(side+"_shoulder_pitch_joint")] for side in ("left","right")]
            title="Training sim2sim reference"
        axes[0,col].set_title(title)
        for row,(y,label,color) in enumerate(((waist,"Waist flexion (deg)",blue),(base,"Pelvis pitch (deg)",orange),(chest,"Chest pitch estimate (deg)",green))):
            axes[row,col].plot(x,np.degrees(y[ix]),color=color)
            axes[row,col].set_ylabel(label)
        for arm,side,color in zip(arms,("Left","Right"),(blue,orange)):
            axes[3,col].plot(x,np.degrees(arm[ix]),color=color,label=side)
        axes[3,col].set_ylabel("Shoulder pitch (deg)")
        axes[3,col].set_xlabel("Time in comparison window (s)")
        axes[3,col].legend(fontsize=9)
    fig.suptitle("Posture and arm swing; chest estimate assumes model-aligned IMU axes",fontsize=14)
    save(fig,"posture_arms.png")
    print("Saved four scientific plots to",args.out)


if __name__=="__main__":main()
