# AMP 按 Guide 后跌倒：关节顺序审计与复现

分析日期：2026-10-12。审计基线为部署分支 `feat/amp-deploy` 的父仓库 `c85d8d8`、`motion_control` 子模块 `d8dd953`。训练仓库 `/Users/yumx/code/X1_29_AMP`。下述错误映射和对照指修正前版本；本次修正与复验见第 4 节。

## 结论

审计基线存在确定的观测/动作关节顺序错误：`rl_walk_amp.joint_list` 使用 MuJoCo/GMR 串行关节顺序，策略实际使用 Isaac Lab 的交错关节顺序。29 个索引的关节语义全部不同。这一错误在保持模型、策略、频率、起始姿态和指令一致的闭环对照中，足以导致约 1 秒内跌倒。

Guide 触发 `/amp_mode`；当前 `settle_ticks=0`，策略从第一拍接管，所以按键后立即暴露错误映射。修正前操作说明中的“1 秒软启动”与当前配置不一致，但本轮证据表明应优先修正关节顺序。

本报告依据审计基线和本地闭环实验。尚未取得这次真机跌倒的日志或现场运行目录，最终仍需确认真机加载的是此版本，并核对修正后的硬件闭环。

## 1. 两套顺序被混淆

训练侧的权威顺序在 `roboparty_train/robolab/scripts/tools/retarget/config/x1.yaml`：

- `gmr_dof_names`：腰 3、左臂 7、右臂 7、左腿 6、右腿 6。
- `lab_dof_names`：Isaac Lab 验证的交错顺序，策略输入和输出使用此顺序。

训练侧 `sim2sim/mujoco_rollout.py:448` 明确读取 **`lab_dof_names`**，再按名称建立 `lab2mj`，用于位置/速度观测和动作目标重排。

部署侧 `motion_control/module/control_module/cfg/rl_x1.yaml:421` 和 `rl_x1_sim.yaml:420` 却使用 **`gmr_dof_names`**，注释误写为训练 LAB 顺序。C++ 在 `rl_controller.cc:623` 直接拼接该顺序下的关节位置/速度，在 `rl_controller.cc:434` 直接将 `actions_[ii]` 下发给 `joint_names_[ii]`，没有策略顺序的置换。

部分动作索引的实际含义如下（索引从 0 开始）：

| 策略索引 | 训练中对应的关节 | 修正前部署下发到的关节 |
|---|---|---|
| 0 | left_hip_pitch_joint | lumbar_yaw_joint |
| 2 | right_hip_pitch_joint | lumbar_pitch_joint |
| 9 | left_knee_pitch_joint | left_wrist_roll_joint |
| 13 | left_ankle_pitch_joint | right_elbow_pitch_joint |
| 16 | right_ankle_pitch_joint | right_wrist_roll_joint |
| 17 | left_ankle_roll_joint | left_hip_pitch_joint |
| 20 | right_ankle_roll_joint | left_knee_pitch_joint |
| 21 | left_elbow_pitch_joint | left_ankle_pitch_joint |
| 22 | right_elbow_pitch_joint | left_ankle_roll_joint |
| 27 | left_wrist_roll_joint | right_ankle_pitch_joint |

`/joint_states` 按名称转发只能保证反馈被写入正确的物理关节；不能保证拼接后的 ONNX 向量符合策略索引。维度检查也无法识别此错误，因为错误顺序仍然是 29 关节、288 维输入。

另有两个同源错误：

1. 训练仓库的参考部署文件 `sim2sim/deploy_onnx_rt.py:48` 自身也把 GMR 顺序写为 `LAB_DOF_ORDER`。该常量不是训练侧通过 sim2sim 使用的顺序，应以验证过的 `lab_dof_names` 和实际 rollout 为准。
2. `scripts/amp_sim_bisect/rollout_bisect.py:484` 在 F1 根目录运行时，找不到训练 YAML，回退到同样错误的 GMR 顺序；指定训练仓库为 `--repo-root` 才会读取正确顺序。这能解释此前本地跨仓库测试通过、便携脚本或部署失败的差异。

## 2. 模型文件没有损坏或自带重排

- 部署 ONNX SHA256：`3e09b76f1e38085a2093abccc1e6f80c86751c9758f7f21d0f497163c99ebd00`。
- F1 与训练仓库中的 `soup59c_50.policy.npz` SHA256 均为 `a0cbe051220bf957ba6947851774541a54b83bc3dc258bd40a6a00f08e2e7b07`。
- 读取部署 ONNX 的初始化张量后，`mean`、`std`、四层权重和偏置与 NPZ 全部逐元素相等，最大误差为 0。
- ONNX 运算为 `Sub → Div → Gemm/Elu × 3 → Gemm → Identity`，图中没有关节重排。训练导出程序 `sim2sim/export_onnx.py` 同样直接导出原始 actor。

因此该 ONNX 仍然要求 Isaac Lab 顺序，不能直接按 GMR 顺序输入和执行。

## 3. 单变量闭环对照

统一条件：macOS arm64、MuJoCo **3.12.0**、同一 `deploy_flat` XML、同一 NPZ、浅蹲 stand 出生态、零历史、`settle=0`、策略 50 Hz、PD 每 1 ms 更新。核心对照只改变关节顺序；`deploy_flat` 路径在脚本中是绝对路径，修改 `--repo-root` 不会改变加载的机器人模型。

| 策略关节顺序 | 目标角限位 | 指令 | 结果 |
|---|---|---|---|
| 修正前部署/GMR | 无额外目标截断 | (0,0,0) | t=0.80 s 检测跌倒，计数 41/400 拍 |
| Isaac Lab | 无额外目标截断 | (0,0,0) | 400/400 拍，8 s；平均平面速度误差 0.019 m/s |
| 修正前部署/GMR | F1 YAML 目标限位 | (0,0,0) | t=1.02 s 检测跌倒，计数 52/400 拍 |
| Isaac Lab | F1 YAML 目标限位 | (0,0,0) | 400/400 拍，8 s；平均平面速度误差 0.021 m/s |
| Isaac Lab | F1 YAML 目标限位 | (1,0,0) | 400/400 拍，8 s；平均平面速度误差 0.103 m/s |

脚本将检测到跌倒的那一拍计入 `survived_steps`，故计数时间比 `[FALL]` 输出大 20 ms。模拟跌倒判据为基座高度低于 0.30 m 或倾角大于 60°；真机 AMP 的倾角保护为 45°。

首拍从同一理想浅蹲姿态、零速和零历史计算出的物理关节目标如下。错误顺序同时影响观测和输出，因此两列不是简单交换同一个动作向量：

| 关节 | 当前姿态 rad | 错误映射原始目标 rad | 正确映射原始目标 rad |
|---|---:|---:|---:|
| left_ankle_pitch_joint | -0.21000 | +0.19688 | -0.23340 |
| left_ankle_roll_joint | 0 | +0.48072 | -0.01135 |
| right_ankle_pitch_joint | -0.21000 | -0.64902（截断为 -0.41） | -0.12049 |
| right_ankle_roll_joint | 0 | -0.25080 | +0.08153 |
| left_knee_pitch_joint | +0.49000 | +0.72033 | +0.36547 |

例如错误映射要求左踝 roll 从 0 一拍跳到约 27.5°。这与“按 Guide 后立刻失稳”相符。

这些实验隔离并验证了策略映射错误；模拟执行采用理想关节空间 PD，未覆盖真实 DCU、并联踝传动和电机动态。

### 复现核心对照

以下对照使用修正前父仓库 `c85d8d8` 的脚本，在 F1 根目录执行，Python 安装 `mujoco==3.12.0` 和 NumPy。第一条读错误回退顺序，第二条读训练的正确顺序；修正后的两条命令均使用 Lab 顺序，不能再用于复现错误映射：

```bash
python scripts/amp_sim_bisect/rollout_bisect.py \
  --ckpt scripts/amp_sim_bisect/soup59c_50.policy.npz \
  --repo-root . --robot deploy_flat --no-align \
  --spawn-pose stand --settle 0 --cmd 0 0 0 --duration 8 --pd-every 1

python scripts/amp_sim_bisect/rollout_bisect.py \
  --ckpt scripts/amp_sim_bisect/soup59c_50.policy.npz \
  --repo-root /Users/yumx/code/X1_29_AMP --robot deploy_flat --no-align \
  --spawn-pose stand --settle 0 --cmd 0 0 0 --duration 8 --pd-every 1
```

注意：修正前脚本的调试打印 `legs_act=act[17:29]` 也假定 GMR 顺序；在正确 Lab 顺序下该切片不是腿部关节集合，不能按标签解释。

## 4. 最小修正范围

现有控制器和 DCU 均按名称归并命令，因此可只将两个 YAML 中 **`controllers.rl_walk_amp` 的 `joint_list`、`init_state`、`stiffness`、`damping` 同步重排** 为 Lab 顺序。仅改变关节名称会导致默认角和 PD 参数错配。

本次已将真机/仿真两份配置的四个字段同步重排；逐名称比较确认默认角、stiffness、damping 数值保持不变，顶层顺序及其他控制器保持原配置。修正后的四个字段如下：

```yaml
joint_list: [left_hip_pitch_joint, lumbar_yaw_joint, right_hip_pitch_joint,
             left_hip_roll_joint, lumbar_roll_joint, right_hip_roll_joint,
             left_hip_yaw_joint, lumbar_pitch_joint, right_hip_yaw_joint,
             left_knee_pitch_joint, left_shoulder_pitch_joint, right_shoulder_pitch_joint,
             right_knee_pitch_joint, left_ankle_pitch_joint, left_shoulder_roll_joint,
             right_shoulder_roll_joint, right_ankle_pitch_joint, left_ankle_roll_joint,
             left_shoulder_yaw_joint, right_shoulder_yaw_joint, right_ankle_roll_joint,
             left_elbow_pitch_joint, right_elbow_pitch_joint, left_elbow_yaw_joint,
             right_elbow_yaw_joint, left_wrist_pitch_joint, right_wrist_pitch_joint,
             left_wrist_roll_joint, right_wrist_roll_joint]
init_state: [0.48891, 0, -0.48891, 0.06213, 0, -0.06213,
             -0.33853, 0, 0.33853, 0.63204, 0, 0, 0.63204,
             -0.27224, 0, 0, -0.27224, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
stiffness: [120, 120, 120, 100, 120, 100, 100, 150, 100, 150,
            40, 40, 150, 50, 40, 40, 50, 50, 40, 40, 50,
            30, 30, 20, 20, 15, 15, 15, 15]
damping: [4, 4, 4, 3.3, 4, 3.3, 3.3, 5, 3.3, 5,
          2, 2, 5, 2.5, 2, 2, 2.5, 2.5, 2, 2, 2.5,
          1.5, 1.5, 1, 1, 1, 1, 1, 1]
```

本次也修正便携 harness 的回退顺序和腿部调试索引，并校验外部训练 YAML 的顺序；修正文档中的错误顺序和“1 秒软启动”说明。训练仓库的参考部署常量未在本次 F1 提交中修改，后续还应将顺序写入模型元数据并在 C++ 加载时校验。当前形状检查不足以保护关节语义。

修正后的脚本仅依赖 F1 仓库，`--repo-root . --robot deploy_flat --no-align --spawn-pose stand --settle 0 --duration 12 --pd-every 1 --via-actuator` 复验：

| 指令 | 存活 | 平均平面速度误差 | 平均偏航速度误差 | 平面位移 |
|---|---|---|---|---|
| (0,0,0) | 600/600 拍，12 s | 0.013 m/s | 0.009 rad/s | 0.04 m |
| (1,0,0) | 600/600 拍，12 s | 0.116 m/s | 0.153 rad/s | 10.68 m |

该复验启用部署 XML motor 通道和 ctrlrange 限幅，但 harness 仍采用被动 damping=1、armature=0.01，未完整复刻 C++ 的目标限位、保护和 ROS2 调度。完整 Linux `run_sim.sh` 集成未在 macOS 本机执行，需按 [服务器说明](AMP仿真服务器运行.md) 验证。

重建后应核对实际启动目录中的 `cfg/control_module/rl_x1.yaml` 和 ONNX 哈希，再在吊架条件下比较第一拍位置目标、踝关节力矩及 IMU。已有日志默认在运行目录的 `test_logs/data_csv/walk_diag_*.csv`，观测日志在 `test_logs/data_csv/t_m/`。

## 5. 后续需要处理的独立问题

- Guide 按住时手柄以 20 Hz 持续发布 `/amp_mode`；修正前 `walk_amp.pre_states` 包含自身，普通模式 1 秒节流之后可以再次 `RestartController()`，清空历史和动作。本次已在两份配置中移除自身前置状态，拒绝同状态重入；退出后需从 stand 再进入 AMP。
- 当前 `settle_ticks=0`，没有 1 秒软启动，本次已更新操作说明；应按立即接管的契约验证切入过程，避免把增加静持时间当作映射错误的修复。
- 真机 IMU 没有显式安装姿态变换，DCU 直接转发四元数和陀螺轴；其与机身坐标的关系仍需现场确认。
- 踝关节力矩限幅为 ±80 Nm，但并联映射后电机为 R52，MIT 编码范围为 ±50 Nm，`PowerFlowR::SetMitCmd` 编码前没有限幅，超范围可能发生位截断/串位。该问题应在映射修正后的电机侧输出检查中核实；本轮尚未证明此次首拍触发了编码溢出。

以上独立问题不能替代已通过单变量实验验证的主要原因：策略与部署的关节顺序不一致。
