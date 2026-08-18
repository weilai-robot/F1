# SPI / SPI-Active Sim2Real Pipeline for X1

基于 **SPI-Active**（*Sampling-Based System Identification with Active Exploration for Legged Robot Sim2Real Learning*, CoRL 2025 Oral, arXiv:2505.14266）的 X1 系统辨识与 sim2real 流水线。

> 完整设计（论文精读 + F1 适配决策）见 [`doc/sim2real_spi.md`](../doc/sim2real_spi.md)

## 流水线总览

```
真实日志 (motion_control/czy/**/*.csv, 100Hz)
   │ prepare_dataset.py          变长 clips (H~U(1s,2s)), 初态对齐
   ▼
sim2real/data/x1_clips.npz
   │ run_spi.py                  Optuna CMA-ES × MuJoCo 开环回放
   │                             log-Cholesky (m,r,I) + tanh 电机 κ + κs
   ▼
logs/spi_sysid/gm_play/identified_params.json|.pt
   │ apply_params.py             回写 URDF/MJCF + 生成 DR 配置
   ▼
sim2real/export/{x1_identified.urdf, xyber_x1_identified.xml, dr_x1_spi.json}
   └─→ gradmotion 远端重训（agibot_x1_train 框架）→ 新 ONNX → 部署
```

Stage-2（可选迭代）：`run_active.py` 用 FIM(tr F⁻¹)+Bézier 命令优化生成最优激励命令序列，真机执行后再回到 `run_spi.py`。

## 执行方式：一切走 gradmotion 远程（本仓库不本地装依赖）

### 一键远程

```bash
# startScript（已含 pip 依赖安装 + 全流水线）:
gm-run F1/sim2real/scripts/remote_sysid.sh
```

`remote_sysid.sh` 在远端镜像内完成：定位兄弟目录 `Humanoid_motion`（含真实数据/MJCF）→ 软链为 `motion_control` → `pip install mujoco optuna pyyaml matplotlib` → 依次跑 prepare/run_spi/mass_landscape/apply_params。产物落在 `logs/`（gradmotion SDK 扫描上传）。

### 本地（仅 numpy 级，无需安装）

```bash
python3 -m unittest discover -s sim2real/tests   # 26 个单元测试
python3 sim2real/scripts/prepare_dataset.py ...  # 数据准备仅需 numpy+pyyaml
```

MuJoCo rollout / CMA-ES 优化需要 `mujoco optuna`，按上面远程方式跑。

## 目录结构

```
sim2real/
├── configs/x1_spi.yaml        # 参数空间/代价权重/clip/active 全部可配
├── spi/
│   ├── param_space.py         # log-Cholesky ↔ (m,r,I)；tanh 电机模型
│   ├── dataset.py             # 真实 CSV → clips（并联踝=力矩指令，串联=位置指令）
│   ├── rollout.py             # MuJoCo 开环回放（初态对齐/惯量注入）
│   ├── cost.py                # 论文 Table 3 代价（无动捕项可关）
│   └── optimizer.py           # Optuna CMA-ES 驱动
├── active/
│   ├── fim.py                 # 有限差分 FIM（delta_param/ksync_steps）
│   ├── bezier.py              # Bézier 命令重参数化
│   └── command_opt.py         # tr(F⁻¹)+终止惩罚 命令优化
├── scripts/                   # prepare/run_spi/mass_landscape/apply/active/remote_sysid.sh
└── tests/                     # numpy 级单测（26）
```

## 关键适配（X1 vs 论文）

| 项 | 决策 |
|---|---|
| 无动捕 | base_pos/base_linvel 代价默认 0，保留 IMU 姿态/角速度 + 关节全项 |
| 仿真器 | MuJoCo（仓库既有 MJCF），非 Isaac Gym |
| 执行器 | 串联髋/膝：驱动器 PD 回放；并联踝：τ_des_lpf 直接回放；统一过 κ·tanh 饱和 |
| 辨识对象 | 骨盆 link_base (4.30 kg) + 4 组电机 κ（hip_pitch/hip_rolleyaw/knee/ankle）+ κs |
| 命令空间 | 当前 3 维 cmd_vel；接入 WTW 式多行为策略后扩展 Stage-2 |

## 远端重训衔接

`apply_params.py` 产出的 `x1_identified.urdf` 替换 `agibot_x1_train/resources/robots/x1/urdf/x1.urdf`，`dr_x1_spi.json` 的范围替换 `x1_dh_stand_config.py` 的 `domain_rand` 段（以辨识值为中心的窄带 DR，论文 nominal range 策略），随后按常规流程创建 gradmotion 训练任务。
