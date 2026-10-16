# AMP 部署就绪排查（2026-10-16）

结论：**v70b m16000b 模型及 v68 辨识 PD 已接入真机代码，但部署验收尚未闭环，不能标记为完全就绪。**

本次检查 F1 `9455215`、motion_control `ec29ebe`，依据当前部署文档明确选定的
`model_16000_v70b.pt` / `m16000b.policy.npz`，没有把训练目录中的其他候选权重自动替换进部署。
本次仅新增排查记录、证据和离线探测脚本，未修改控制参数、生产模型或启动流程。

## 已核实的部署内容

- 真机 `rl_x1.yaml` 与仿真 `rl_x1_sim.yaml` 的 AMP 模型均指向
  `x1_amp_v70b_m16000_50hz.onnx`。
- 29 关节 Kp/Kd、default_q 与训练 `mujoco_rollout.py` 的 v68 表逐项相同；
  `check_v68_params.py` 对训练配置、辨识表和训练侧 sim2sim 的检查通过。
- `test_amp_pd_v70b.py` 6/6 通过；MIT 编码测试通过，R52/R86 的 Kd=8 可正确编码。
- ONNX 的四层权重、bias、mean/std 与 `m16000b.policy.npz` 逐项相等。
  200 组随机输入对 NumPy 前向的最大绝对误差为 `6.866455078125e-05`。
- ONNX 输入 `obs: float32[1,288]`，输出 `action: float32[1,29]`，IR=8、opset=17；
  C++ 为 96 维观测 × 3 帧、term-major / oldest-first、50 Hz、动作缩放 0.25，
  绕过旧观测缩放，归一化器在图内。串联关节下发 Kp/Kd，踝关节软件 PD 使用同一配置。
- `scripts/run_amp_real.sh` 的 Bash 语法检查通过；CMake install 规则包含配置和 policy 目录。

模型 SHA256：

```text
93328d2ac25be6001e806c38f839ed0617c01e2cde41a833dfb2276467450b45
```

## 阻碍就绪的发现

### 1. F1 仿真动力学没有同步辨识结果

直接加载 `sim_x1.yaml` 使用的 `xyber_x1_flat.xml`，29 个 hinge 的 armature **全部为 0**。
`SimModule` 从 XML 加载后没有覆盖 armature。训练侧则为 hip_pitch=0.208、
knee_pitch=0.273、ankle_pitch=0.15 等逐关节辨识值。

这影响仿真验收；真实机器人不需要额外下发 armature，物理惯量已经存在。

### 2. F1 仿真力矩上限与训练不一致，闭环探测失败

`xyber_x1_serial.xml` 四个踝关节 motor 的 ctrlrange 为 **±18 Nm**；
训练及 AMP 软件 PD 的踝关节上限为 **±80 Nm**。髋 roll/yaw 的 XML 上限也只有
±50 Nm，而训练为 ±180 Nm。上述是仿真关节力矩上限，不代表实机并联电机可直接使用同一数值。

离线探测使用实际部署 ONNX、F1 flat 场景、浅蹲 stand 初始姿态、固定 `[1,0,0]`、
settle=0、50 Hz 策略、1 ms PD、motor 通道，以及 YAML 目标限位、观测/动作裁剪、45° 倾角阈值。

| 探测条件 | 结果 |
|---|---|
| v68 PD + 当前 XML armature=0，近似 C++ 力矩限幅 | 2.24 s 时超过 45°，未通过 12 s |
| v68 PD + 辨识 armature，近似 C++ 力矩限幅 | 3.02 s 时超过 45°，未通过 12 s |
| v68 PD + 辨识 armature，训练 effort 限幅再叠加 XML 限幅 | 3.08 s 时超过 45° |
| 上一行仅将四个踝 motor 的 XML 上限临时改为 ±80 Nm | **12.00 s / 600 拍通过**，距离 10.74 m，速度误差 0.107 m/s |
| v68 PD + 辨识 armature + 全部训练力矩上限 | 同样通过 12.00 s |

第三、四行只有踝 XML 上限发生变化，证明此限制足以影响该探测的成败。
仅补 armature 不能解决当前 F1 仿真验收问题。生产参数没有据此自动调大；
应先确认真实关节可用力矩，再统一 AMP 仿真配置与训练假设。

**范围限制：上述是 Python/MuJoCo 对照探测，不是完整 C++/AimRT 实测。**
探测不覆盖 ROS2 调度、回调延迟、真实电机内部环、并联踝传动或实际 stand 稳态误差；
`survived_s` 包含检测到超阈值的最后一拍。不能据此宣称实机会在同一时刻跌倒。

### 3. 现有独立闭环脚本仍是旧 PD 和统一 armature

`scripts/amp_sim_bisect/rollout_bisect.py` 的 PD 仍为髋 120/4、膝 150/5 等旧值，
并无条件将 hinge armature 设为 0.01，即使传入新 ONNX 也不会读取部署 YAML 的新 PD。
本次新 ONNX 在这些旧参数下反而能通过 12 s，所以此脚本的通过结果不能验收当前部署参数。

### 4. 启动检查没有验证实际运行中的新 PD/模型

`scripts/run_amp_real.sh:80` 仅检查文件存在及 YAML 是否含新模型文件名，
没有验证构建目录中的 Kp/Kd、关节序和模型 hash；已有 tmux 会话会被复用。
因此旧 PD 配上新文件名、或更新磁盘后未重启进程，均不能由该检查排除。
已有源码守卫只检查源码 YAML，不检查构建产物和运行中实例。

### 5. 真机反馈问题尚缺复验

10-13 左臂 CAN 支路成组冻结的问题，在现有记录中仍未获得真机复验结论。
`AmpReady()` 仅检查整条 joint_states/IMU 消息的到达时间；DCU 持续发布含重复值的
消息时，这不能证明每个电机都在返回新测量。更新策略不能替代该链路的验证。

## 完成就绪验收所需工作

1. 为 AMP 建立与辨识参数一致的仿真配置，并依据实机能力统一关节力矩上限；
   独立闭环工具明确读取相同 PD/armature/限幅，保留旧模型测试的版本边界。
2. 用部署 ONNX 通过 F1 模型的闭环回归，再执行 Linux 上完整 `run_sim.sh` 集成。
3. 真机重新构建和重启，检查实际构建目录及加载实例的模型 hash、PD、关节序，
   取得左臂反馈和新 PD 的诊断日志后完成受保护的实机验收。

当前本机为 macOS，无项目 Linux ROS2 构建产物，本次没有执行完整构建或连接机器人。
数值验证使用 ONNX Runtime 1.29.0；部署附带库为 1.19.2，其实际加载尚未复验。

证据：[readiness_evidence.json](analysis/amp_20261016/readiness_evidence.json)。
复现：[readiness_probe.py](analysis/amp_20261016/readiness_probe.py)，需 NumPy、
ONNX Runtime、MuJoCo 3.12.0，以及同级 `X1_29_AMP` 训练仓库；用法见脚本开头。
