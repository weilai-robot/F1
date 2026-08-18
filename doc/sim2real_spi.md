# F1 Sim2Real 改造 — SPI / SPI-Active 方案

> 分支：`dev/sim2real-spi`
> 论文：**Sampling-Based System Identification with Active Exploration for Legged Robot Sim2Real Learning**（SPI-Active, CoRL 2025 Oral, CMU LeCAR Lab）
> arXiv: [2505.14266](https://arxiv.org/abs/2505.14266) · 开源库: [LeCAR-Lab/SPI-Active](https://github.com/LeCAR-Lab/SPI-Active)（MIT, Isaac Gym Preview 4）

---

## 1. 论文精读摘要

### 1.1 定位与动机

Sim2real 差距主要来自物理参数失配（质量、质心、惯量、摩擦、执行器未建模效应）。四类弥合手段中：

| 手段 | 缺点 |
|---|---|
| 域随机化 (DR) | 依赖启发式调参；过宽→策略保守，过窄→泛化差 |
| 黑盒系统辨识（学残差模型） | 任务相关、易过拟合、常需真值力矩 |
| 自适应策略学习 | 常需高质量在线数据、非零样本 |
| **白盒系统辨识（本文）** | 直接估计物理意义参数，可解释、可泛化 |

传统白盒 SysID 的困难：腿足系统接触不连续、强非线性，现有方法要么要求**可微仿真器**，要么要求**真值力矩传感器**，要么只能辨识少量参数。SPI 用**大规模并行采样 + 零阶优化**绕开这些假设。

### 1.2 SPI（Stage 1）：基于采样的参数辨识

**问题形式化**：参数化动力学 `x_{t+1} = f(x_t, u_t; θ)`，给定真实数据集 `D = {(x_t, u_t)}`，解

```
θ* = argmin_θ Σ_t ‖x_{t+1} − f(x_t, u_t; θ)‖²
```

**辨识参数** `θ = [θ_in, θ_mo]`：
- `θ_in`：**单个刚体**（base link）的质量 m、质心 r∈R³、惯量 I（3×3 对称）——论文辨识 Go2 躯干 / G1 骨盆，方法可扩展到更多 link
- `θ_mo`：电机模型参数（见下）

**质量-惯量的 log-Cholesky 参数化**（保证伪惯量阵 J(θ_in) ≻ 0 物理可行）：

```
J(θ_in) = U Uᵀ,   U = e^α · [[e^{d1}, s12, s13, t1],
                              [0,     e^{d2}, s23, t2],
                              [0,     0,     e^{d3}, t3],
                              [0,     0,     0,    1  ]]
φ = [α, d1..d3, s12, s13, s23, t1..t3] ∈ R¹⁰   ← 无约束优化变量
```

由 φ 可解析恢复 m、r = h/m、I = tr(Σ)I₃ − Σ 等。这一步对采样式优化至关重要——避免采出非法惯量。

**执行器模型**（tanh 饱和，源自 [62]）：

```
τ_motor = κ · tanh(τ_PD / κ),    τ_PD = Kp(q_target − q) − Kd·q̇
```

- 小指令时 τ_motor ≈ τ_PD（不扰动），大指令平滑饱和——刻画真实电机高力矩区间的力矩衰减
- κ 按关节分组给定（Go2: κ_Hip/κ_Thigh/κ_Calf；论文还带线性增益 κs∈[0.5,1.5]）

**数据采集与预处理**：
- 用启发式运动脚本 + 预训练 RL 策略上真机采数据（运动先验）
- 按 simulation error criterion 切成 clips `{c_k}`，horizon `H ~ U(H_min, H_max)`（验证集平均 1.5 s）——变长避免固定视野偏差

**多步预测代价（H-Step Sequential Prediction）**：

```
J(θ, {c_k}) = Σ_k Σ_{t=0}^{H-1} ‖x^r_{t+1,k} − x_{t+1,k}‖²_Wx + ‖θ − θ₀‖²_Wθ
```

每个 clip 的**初态与真实轨迹对齐**，之后只回放录制的控制量 `u^r_t`。论文 Table 3 代价项：

| 类别 | 项 | 表达式 | 系数 | 全局缩放 |
|---|---|---|---|---|
| 基座预测 | 位置 | ‖p−p_r‖² | 4.0 | — |
| 基座预测 | 速度 | ‖v−v_r‖² | 2.0 | ×0.5 |
| 基座预测 | 姿态 | 1−⟨q,q_r⟩² | 2.0 | — |
| 基座预测 | 角速度 | ‖ω−ω_r‖² | 0.5 | ×0.5 |
| 关节预测 | 关节位置 | ‖q_jnt−q_jnt,r‖² | 3.0 | — |
| 关节预测 | 关节速度 | ‖q̇_jnt−q̇_jnt,r‖² | 0.1 | — |
| 关节预测 | 关节力矩 | ‖τ−τ_r‖² | 0.01 | ×0.2 |
| 参数正则 | 质量 | ‖m−m₀‖² | 0.01 | ×0.1 |
| 参数正则 | 质心 | ‖r−r₀‖² | 10.0 | ×0.1 |
| 参数正则 | 惯量 | ‖I−I₀‖² | 1.0 | ×0.1 |
| 参数正则 | tanh 增益 | ‖κ−κ₀‖² | 0.01 | ×0.1 |
| 参数正则 | 线性增益 | ‖κs−κs,₀‖² | 0.1 | ×0.1 |

**优化器**：CMA-ES（Optuna 实现，Gaussian sampler，论文跑 5 iterations）；每代并行采样 batch `{θ_j}`，GPU 仿真器并行 rollout 打分。参数采样范围（Table 4）：

| 参数 | Go2 躯干 | G1 骨盆 |
|---|---|---|
| 质量 m | 3–15 kg | 1–10 kg |
| 惯量对角 | (0.005,…,0.005)–(1.0,…,1.0) | 同左 |
| 质心 r | ±0.1 m | ±0.2 m |
| κ_tanh | 10–40 | — |

### 1.3 SPI-Active（Stage 2）：主动探索数据采集

**动机**：启发式数据无法充分激励所有参数（论文实验：SPI 在姿态跟踪任务上欠佳，正是姿态相关参数激励不足）。

**原理**：Cramér-Rao 下界 `Cov(θ̂) ⪰ F(θ*)⁻¹`，最小化 `tr(F⁻¹)` 即压低估计方差下界。高斯过程噪声假设下：

```
F(θ,π) ≈ σ⁻² · E[ Σ_t (∂f/∂θ)(∂f/∂θ)ᵀ ]
```

**关键设计——层级式命令优化而非直接学探索策略**（直接学探索策略在腿足上会失控）：
- 取**预训练多行为策略** `π(u_t | x_t, c_t)`（walk-these-ways 式，Go2 命令 14 维：`[vx,vy,wz, h, f, b1..b4, hf, φ,ψ, sw, sl]`）
- 只优化**命令序列** `c_{1:T}` 的子集 `[vx, vy, ωz, b1, b2, φ, ψ]`，其余固定
- 命令轨迹用 **10 阶 Bézier 曲线**重参数化（压缩优化变量数），按 **H=4 s** 分段；步态命令 (b1,b2) 从 4 个离散组合（pace/trot/bound/pronk）中选
- FIM 梯度用**有限差分**（仿真器不可微）；**终止惩罚**防摔
- 同一 CMA-ES 求解；θ* 用 Stage-1 估计 θ̂₁ 替代

**实现细节（开源库）**：`delta_param=0.1`（FD 扰动步长）、`ksync_steps=5`（**辅助环境每 5 步重同步到主环境状态**，保证 FD 测的是局部敏感度而非发散轨迹差）、Bézier 控制点数可配（`num_bezier_points`）、`command_sampling_idxs` 可选维度。

### 1.4 下游任务训练与部署

- 辨识参数回写 URDF，DR 用 **nominal range**（Table 8）：基座质量 ×U(0.8,1.2)、CoM ±0.1、惯量 offset ±0.05、κ 按关节组窄带（如 κ_Hip U(22,24)）
- PPO + 非对称 actor-critic（critic 拿特权信息），Isaac Gym；动作=目标关节位置→PD
- 观察：ω、重力投影、q、q̇、上一步动作（+任务特定）
- 结果：SPI 比 Vanilla 提升 19.6–39.9%，SPI-Active 全任务最优、超基线 42–63%（Go2 挂 4.7kg 载荷 + G1 人形速度跟踪，**开环无全局位置反馈**）

---

## 2. 开源库结构（LeCAR-Lab/SPI-Active）

```
SPI-Active/
├── spigym/
│   ├── simulator/isaacgym/isaacgym_sysid.py          # 并行 SysID rollout
│   ├── simulator/isaacgym/isaacgym_active_sysid.py   # Stage-2 辅助环境 + FIM
│   ├── agents/sysid/active_sysid.py                  # Optuna CMA-ES 命令优化主体
│   ├── envs/sysid/                                   # SysID 环境
│   ├── config/robot/g1/g1_23dof_sysid.yaml           # G1 sysid 配置
│   ├── config/algo/active_sysid.yaml                 # 命令空间/采样模式/优化器
│   └── config/domain_rand/*.yaml                     # nominal/heavy DR 范围
├── scripts/
│   ├── mass_landscape.py                             # 质量-代价地形（诊断）
│   ├── mass_opt.py                                   # 单参数 Optuna 辨识
│   └── data/{walk,jump,stand,sine}.py                # 数据采集脚本
└── active_sysid.md                                   # 二阶段使用文档
```

依赖 Isaac Gym Preview 4（Linux + NVIDIA），仅此一点即不能直接跑在本仓库环境——需按 §4 做适配。

---

## 3. F1 / X1 现状盘点

| 项 | 现状 |
|---|---|
| 机器人 | X1 人形，29 关节（3 腰 + 14 臂 + 12 腿），RL 行走策略控 12 腿关节 |
| 策略部署 | `motion_control/module/control_module`，ONNX（rl_walk_leg，obs 47+hist 66 → act 12），100 Hz 推理（decimation 10 @1 kHz），action_scale 0.5 |
| 驱动方式 | **髋/膝串联电机：下发位置**，驱动器 PD（walk kp=[30,40,35,100,35,30]×2, kd=[3,3,4,8,1.5,1.5]×2）；**踝并联电机：软件下发 effort 力矩**（`rl_controller.cc:28`），踝目标位置经 100 Hz LPF |
| 仿真模型 | `sim_module/model/mjcf/robot/xyber_x1/xyber_x1_serial.xml`（MuJoCo，骨盆 link_base m=4.304 kg，fullinertia，力矩电机 ctrlrange） |
| 真实数据 | `motion_control/czy/real_data/round_exp_*.csv`（1 kHz 激励实验：kp40/kd3/ff0 等扫参）、`czy/8.7/walk_diag_*.csv`（行走诊断）。字段：IMU quat/gyro/accel、关节 pos/vel/effort、pos_des_raw/lpf、tau_des_raw/lpf、is_parallel、contact、cmd、phase |
| 训练平台 | gradmotion（外部 RL 训练平台，gm CLI；本仓库策略均在远端训练） |
| sim2real 现状 | 无系统辨识环节：策略在名义 URDF + 经验 DR 下远端训练，真机表现依赖手工调 DR 与 PD |

**关键差距（X1 vs 论文实验设置）与适配决策**：

| 差异点 | 论文（Go2/G1） | X1 适配 |
|---|---|---|
| 基座位置/速度真值 | 动捕 | **无动捕** → 丢弃基座位置/线速度代价项，保留 **IMU 姿态(quat) + 角速度(gyro)** 代价 + 全部关节代价（论文本身指出关节代价只需本体感知） |
| 仿真器 | Isaac Gym（GPU 并行） | **MuJoCo**（`mujoco.rollout` C 级批量 rollout + 多线程；X1 已有 MJCF）；批量不够时可远端跑 |
| 执行器 | 全关节位置指令 + κ·tanh 模型 | 串联关节同论文（PD 在驱动器，q_des 已录）；**并联踝直接力矩指令**：τ_des_lpf 即 u，仍过 tanh 饱和模型 |
| 数据采集 | 预训练多行为策略 + 运动脚本 | 已有 round_exp 激励实验（扫 kp/kd/ff）+ walk_diag 行走日志；后续用 Stage-2 优化命令再采 |
| 命令空间 | 14 维（WTW 式） | 当前策略 3 维 cmd_vel `[vx,vy,wz]` → Stage-2 默认优化全部 3 维 |
| 辨识对象 | Go2 躯干 / G1 骨盆 | X1 **link_base（骨盆）**（可扩展 link_lumbar_pitch 躯干：腰关节之上含双臂 9.08 kg）+ 电机 κ（hip/knee/ankle 三组）|

---

## 4. F1 改造设计

### 4.1 流水线

```
┌──────────────── 真机数据 ────────────────┐
│ round_exp_*.csv / walk_diag_*.csv (1kHz) │
└──────────────────┬───────────────────────┘
                   │ prepare_dataset.py（切 clips, H~U(Hmin,Hmax), 初态对齐）
                   ▼
             data/x1_clips.npz
                   │
                   │ run_spi.py —— Optuna CMA-ES
                   │   每个候选 θ: log-Cholesky→(m,r,I) + κ组
                   │   MuJoCo 开环回放 u_t → 多步预测代价(Table 3 权重)
                   ▼
        export/identified_params.json  ──→  mass_landscape.py（诊断地形）
                   │ apply_params.py
                   ├─→ export/xyber_x1_identified.xml（回写惯量的 MJCF）
                   ├─→ export/dr_x1_spi.yaml（nominal DR，以辨识值为中心）
                   └─→ gradmotion 远端重训（gm task create，按 skill 模板）
                   │
                   ▼
           新 ONNX → motion_control policy/ 部署
                   ▲
┌── Stage 2（可选迭代）───────────────────┐
│ run_active.py：FIM(tr F⁻¹) + Bézier 命令优化 │
│ → 最优命令序列上真机再采 → 回到 run_spi.py  │
└─────────────────────────────────────────┘
```

### 4.2 目录结构（本分支新增）

```
sim2real/
├── README.md              # 使用手册
├── requirements.txt       # mujoco, optuna, numpy, matplotlib
├── configs/x1_spi.yaml    # 参数空间/代价权重/clip 配置（全部可覆写）
├── spi/
│   ├── param_space.py     # log-Cholesky ↔ (m,r,I)；tanh 电机模型；参数空间定义
│   ├── dataset.py         # CSV → clips → npz（含踝并联/串联指令选择、LPF 通道）
│   ├── rollout.py         # MuJoCo 批量开环回放（初态对齐、按参数修改惯量）
│   ├── cost.py            # Table 3 代价（含全局缩放、可开关无动捕项）
│   └── optimizer.py       # Optuna CMA-ES 驱动器
├── active/
│   ├── fim.py             # 有限差分 FIM（delta_param, ksync_steps 重同步）
│   ├── bezier.py          # Bézier 命令重参数化
│   └── command_opt.py     # tr(F⁻¹)+终止惩罚 的命令序列优化
├── scripts/
│   ├── prepare_dataset.py
│   ├── run_spi.py
│   ├── mass_landscape.py
│   ├── apply_params.py
│   ├── run_active.py
│   └── remote_sysid.sh    # gradmotion 远端引导：装依赖→定位 Humanoid_motion→全流水线
└── tests/                 # 本地单元测试（numpy 级必过；mujoco 级自动跳过）
```

### 4.3 关键实现约定

1. **参数空间**（`configs/x1_spi.yaml`，初值取自 MJCF 名义值）：
   - link_base：log-Cholesky φ∈R¹⁰；范围质量 2–10 kg、质心 ±0.15 m、惯量对角 0.005–1.0 kg·m²（参照 G1 骨盆 Table 4 等比缩放）
   - 电机 κ_tanh：hip / knee / ankle 三组，范围 10–40（论文 Go2 段）初始取名义 τ_max 附近；κs 线性增益 0.5–1.5
2. **回放控制**：髋/膝 `τ_PD = kp(q_des_lpf − q) − kd·q̇`（kp/kd 从数据文件名或 rl_x1.yaml 注入）→ `κ·tanh(τ_PD/κ)`；并联踝 `τ = κ·tanh(τ_des_lpf/κ)`。**q 取仿真状态**（非录制值），保证是真正的闭环执行器模型
3. **代价**：默认启用 quat(2.0)/ω(0.5×0.5)/q(3.0)/q̇(0.1)/τ(0.01×0.2) + 全部正则(×0.1)；`base_pos/base_lin_vel` 权重默认 0（无动捕），有动捕后改回 4.0/1.0
4. **远程训练衔接**：`apply_params.py` 产出的 MJCF + DR yaml 按 gradmotion 任务模板合入 payload（`gm task create` 遵循 skill 最小模板：taskBaseInfo/taskCodeInfo、goodsId、imageVersion、--file 提交、--dry-run 预检）
5. **执行策略（用户约束）**：**本地不安装任何依赖、一切执行走 gradmotion 远端**。本地仅跑 numpy 级单元测试与纯标准库操作；MuJoCo 回放/CMA-ES 优化由远端任务执行，startScript 固定为 `gm-run F1/sim2real/scripts/remote_sysid.sh`（脚本内部 pip 安装 requirements.txt、软链 Humanoid_motion 为 motion_control、依次跑 prepare→run_spi→mass_landscape→apply，产物写 `logs/` 由 SDK 回传）。详细用法见 `sim2real/README.md`。

### 4.4 局限与后续

- 无动捕 → 基座平移动力学辨识弱（质量/CoM 主要靠关节力矩+姿态误差间接可观）；接入动捕/SLAM 全局位姿后开启 base_pos/base_lin_vel 代价
- Stage-2 需要一个**多行为策略**作为命令载体；当前 X1 仅 3 维 cmd_vel 行走策略，命令子空间小 → 激励多样性受限，建议后续训练 WTW 式多行为策略后再跑 Stage-2
- 踝并联机构在 MJCF 中以两个独立铰链近似，残余模型误差会部分被 κ_ankle 吸收
- 摩擦/接触参数暂不在辨识空间（论文也只辨识惯量+电机）；可作为扩展维度

---

## 5. 首轮远端运行结果（2026-08-18，TASK_20260818_041）

gradmotion A10 (ESKU000004) + Isaac Gym 镜像 V000124 (py3.8)，`remote_sysid.sh` 全自动完成。数据：28 clips（round_exp kp40/kd3 ×2 + walk_diag ×1，dt≈10ms）。

| 指标 | 值 |
|---|---|
| nominal 参数预测代价 | **3,039,218** |
| CMA-ES 最优代价 | **118,019**（**↓25.8×**） |
| pelvis 质量 | 4.304 → **6.973 kg**（+62%，吸收电池/装载等未建模质量） |
| κ_tanh（hip_pitch / hip_rolleyaw / knee / ankle） | 62.0 / 22.4 / 150.1 / 22.1（全部在搜索盒内，辨识良好） |
| κs（电机线性增益） | 0.536（真实出力约为标称 54%，提示模型力矩偏乐观） |

**弱可观测方向（工程判断，已在导出时钳制）**：无动捕下 com_y/z（raw ±0.19/−0.20）与惯量（raw eig 至 2.27 kg·m²）超出 7 kg 骨盆的物理合理域——由质量-惯量-增益相关性吸收模型误差。`apply_params.py` 导出 URDF/MJCF 时钳制 com 每轴 ±0.15 m、惯量 eig ∈ [0.005, 1.0]，raw 值完整保留在 `dr_x1_spi.json` 的 `raw` 块与 `report.md` 中（可用 `--no-clamp` 关闭）。

产物（本地 `sim2real/`，均由 numpy 级脚本生成）：
- `results/identified_params.{pt,json}` —— 远端回传的辨识结果
- `export/x1_identified.urdf` / `xyber_x1_identified.xml` —— 回写后的骨盆惯量
- `export/dr_x1_spi.json` —— 以辨识值为中心的 DR 配置（质量 ±5%、com ±0.03 m）
- `export/report.md` —— 人读报告

远端重训衔接：`x1_identified.urdf` → `agibot_x1_train/resources/robots/x1/urdf/x1.urdf`；`dr_x1_spi.json` 范围替换 `x1_dh_stand_config.py` 的 domain_rand 段；κ/κs 用于部署侧力矩限幅与 actuator 模型校准。
