# AMP v70b m16000b 部署包（2026-10-15）

基于 2026-10-13 真机行走日志完成 sim2real 辨识（`X1_29_AMP/x1_armature_pd.md`，
方案 B 定稿），并已用辨识参数（新 armature + 新 kp/kd）续训（v68→v69 m13497→v70
TASK_20261015_065→v70b 谱系）；本次部署将训练最佳模型 **m16000b**
（`model_16000_v70b.pt`，用户指定）连同 **v68 辨识 PD** 一起替换真机配置。

## 变更清单

| 文件 | 变更 |
|---|---|
| `motion_control/module/control_module/policy/x1_amp_v70b_m16000_50hz.onnx` | 新增：m16000b 导出（288→512→256→128→29 ELU，归一化器融合图内） |
| `motion_control/module/control_module/cfg/rl_x1.yaml` | `rl_walk_amp`: stiffness/damping → v68 辨识值；policy_file → 新 ONNX |
| `motion_control/module/control_module/cfg/rl_x1_sim.yaml` | 同上（与真机同契约） |
| `scripts/run_amp_real.sh` | 模型就位检查 → 新 ONNX 文件名 |
| `scripts/tests/test_amp_pd_v70b.py` | 新增守卫：PD 数组/kd≤8/关节序/模型文件存在性 |

PD 对照（策略关节序，`x1_armature_pd.md` §4；旧值→新值）：

| 关节 | kp 旧→新 | kd 旧→新 | 阻尼比 ζ |
|---|---|---|---|
| hip_pitch | 120 → 65 | 4.0 → 8.0 | 0.26 → 0.71 |
| hip_roll | 100 → 60 | 3.3 → 8.0 | 0.19 → 0.60 |
| knee_pitch | 150 → 85 | 5.0 → 8.0 | 0.33 → 0.70 |
| lumbar_yaw/roll/pitch | 120/120/150 → 90/70/100 | 4/4/5 → 8 | 0.2 → 0.4–0.5 |
| ankle_pitch | 50 | 2.5 → 4.0 | 0.45 → 0.73 |
| 肩 pitch/roll | 40 | 2.0 → 4.0 | 0.36 → 0.72 |
| 其余 | 不变 | 按 ζ≈0.7 微调 | — |

所有 kd ≤ 8.0（R86/R52 主机 MIT 编码上限），腕关节不变（真机 L28 内部位置环）。
站立/零位/keep 控制器（pd_stand 等）不在本次范围，保持不变。

## 部署前验证（已在 Mac 本地完成）

1. **参数四向核对**：`X1_29_AMP/roboparty_train/check_v68_params.py` PASS ——
   md 表 ↔ 训练 `x1.py` ↔ sim2sim `mujoco_rollout.py` ↔ 部署数组完全一致；
   ζ/fn 复算一致；200Hz 显式 PD 稳定性（kd·dt/I < 0.8）满足。
2. **权重溯源**：`m16000b.policy.npz` 与 `model_16000_v70b.pt` 重新抽取逐位相等（diff=0）。
3. **ONNX 等价**：随机观测 max|onnx−numpy| = 6.9e-05；v70b 真实轨迹回放逐帧
   max|onnx−numpy| = 7.6e-06（图内含归一化器，喂原始观测）。
4. **部署管线自检**：`deploy_onnx_rt.py --self-test` 回放 walk1.0 日志 600 步全有限。
5. **sim2sim 闭环验收**（npz 权重 + v68 PD + 辨识 armature，
   `X1_29_AMP/acceptance/v70_eval/`）：
   - nominal / noise1x / noise2x / lat1 / lat2 / lag1 / lag2 / mass±15%：**5/5 全过**
   - walk 1.0 m/s：12 s / 10.7 m 无跌倒，vxy 误差 0.076 m/s
   - push0.75：3/5，push1.0：0/5 —— **抗推扰弱是本模型已知短板，实机测试严禁推拽**

## 真机部署步骤（操作员）

1. 同步代码到机器人上位机（或直接在机上 `git pull`）：
   - F1 仓库 `feat/amp-deploy` 分支（含子模块指针更新）
   - 子模块 Humanoid_motion 同分支
2. 构建：`bash scripts/build.sh`（或 `./run_amp_real.sh --build`）
3. 确认构建产物就位（脚本会自动检查，失败会报错退出）：
   - `build/cfg/control_module/policy/x1_amp_v70b_m16000_50hz.onnx`
   - `build/cfg/control_module/rl_x1.yaml` 中 stiffness 首值 = 65.0、damping 首值 = 8.0、
     policy_file 指向新 ONNX
4. 运行：`./scripts/run_amp_real.sh`（zero → stand → 人工确认 → amp，固定 1 m/s 指令）
   ⚠ 首次上腿必须在吊架下；新 kp 明显低于旧值（腿更"软"）、kd 更大，stand→amp 过渡
     手感会不同；急停必须在手边。

## 数据采集清单（每轮）

行走日志**自动**记录：进入 AMP 状态即写 `<build>/test_logs/data_csv/walk_diag_<时间戳>.csv`
（50Hz，含 29 关节 pos/vel/effort、目标、IMU、指令、action、电机侧力矩
—— dcu `actuator_debug: true` 已开启）。

建议采集（与 2026-10-13 对照）：
1. walk 1.0 m/s ≥ 10 s × 2 次（与 soup59c 基线同工况）
2. walk 0.5 m/s ≥ 10 s × 1 次
3. 如状态允许：原地方向小指令（vy/wz）各 5 s
4. 每次用手机拍侧面视频（步态节律/摆臂肉眼对照）

## 回滚

`rl_x1.yaml` policy_file 改回 `x1_amp_soup59c_50hz.onnx`、PD 改回旧值
（120/100/150 系列，见 git 历史），重新 build 即可。旧 ONNX 未删除。

## 已知事项

- 10-13 轮发现的**左臂反馈成组冻结**（CAN channel_1 支路）如未修复，本模型同样
  无法摆臂——v70 训练描述中的 arm-fold 修复是策略侧护栏，不是硬件修复。
  上腿前建议先做一次上肢静态方向测试确认 29 关节反馈全部活跃。
- push 鲁棒性弱（见上），测试中不要施加横向推力。
