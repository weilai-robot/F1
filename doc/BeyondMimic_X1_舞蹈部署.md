# X1 BeyondMimic 舞蹈部署说明

适用分支：`feat/x1-beyondminic-dance-deploy`。训练仓库：`/Users/yumx/code/Beyondminic-Weilai-X1`。本分支提供 X1 29DOF 舞蹈策略的离线资产、50 Hz 追踪控制器、专用 AimRT 配置和状态检查。**腰俯仰的训练关节坐标与真机控制配置疑似相反；分支已加入双向符号映射，但现场标零、机械方向和锚点 TF 尚未验证，`sign_calibration_verified` 默认关闭，`/dance_mode` 会被拒绝。完成 Linux 构建与闭环验证前不可实机舞蹈。**

## 腰俯仰的坐标与限位

训练 URDF 的 `lumbar_pitch_joint` 限位是 `[-0.15, +0.45] rad`，本项目 `motion_control/module/control_module/cfg/rl_x1.yaml` 的真机限位是 `[-0.45, +0.15] rad`。两者恰好互为负号。若直接当作同一坐标比较，647 帧中有 604 帧越界，第 0 帧就到策略坐标的 `+0.45 rad`。**此前把这认作机械限位冲突过于武断：限位数值相反也可能只是坐标方向相反。**

链路核对结果：训练动作、策略观测和策略输出均按训练 URDF 关节坐标；DCU 的电机角进入 `LumbarParallelTransmission` 后，腰俯仰直接取右腰电机位置乘 `direction_right: +1`，发命令时也乘 `+1`。`ControlModule` 只在关节反馈上减 `joint_offset`、在关节目标上加回偏移；腰俯仰偏移当前为 0。其他多个关节的传动系数是 `-1`，但这些系数已经在 `/joint_states` 之前和电机命令之后成对应用；不能再拿它们解释腰俯仰的差异。其余 28 个关节的控制限位与训练 URDF 一致。

舞蹈控制器现在单独配置 `joint_sign.lumbar_pitch_joint: -1`，暂按 `q_policy = - (q_dcu - joint_offset)`、`qd_policy = -qd_dcu` 处理腰俯仰，位置与力矩命令反向换回 DCU 坐标；全局限位、DCU 传动及其他控制模式不改。按这个**尚待实机确认的假设**，训练参考腰俯仰 `[-0.071, +0.45] rad` 对应 DCU 坐标 `[-0.45, +0.071] rad`，落在原有限位内。控制器把 DCU 限位先转换到策略坐标，再逐帧核对参考；不允许静默越界。动作很多帧落在 `-0.45 rad` 机械边界附近，`joint_limit_margin: 0.03` 会限制目标到 `-0.42 rad`，仍需检查跟踪质量。

现场必须在吊架、低能量条件下对比实机腰向前/向后小幅运动、右腰电机原始角、`/joint_states` 腰俯仰角和训练 URDF 的正向，确认零位和方向，再核对两端机械止挡。还要确认 `/dance/anchor_pose` 的 TF 使用**训练 URDF 关节坐标**：若 TF 直接由原始 `/joint_states` 生成，在这个反号假设下腰俯仰一动，锚点姿态就会算错；需要在 TF/FK 输入侧做相同转换并验证。以上均核实后才把 `sign_calibration_verified` 改为 `true`。如果实测正向一致，应把 `joint_sign` 恢复 `+1`，再查明当前限位配置与实机机械范围哪一项错误；不能仅凭 URDF 放宽安全限位。

## 训练与推理契约

| 项目 | 本分支使用值 |
| --- | --- |
| Checkpoint | `trained_models/dance_x1_model_4999.pt`，Actor 160→512→256→128→29，ELU，输出确定性均值动作 |
| 参考动作 | `whole_body_tracking/motions/dance_x1_train.npz`，647 帧、50 Hz；单次约 12.94 秒，到末帧后自动转 `keep`，不循环瞬移 |
| 主循环与策略 | 控制主循环 1000 Hz；每 20 拍推理一次，即 50 Hz |
| 输入 | 参考关节角 29、参考关节速度 29、参考锚点位置误差 3、参考锚点相对旋转前两列 6、机身线速度 3、角速度 3、相对默认角 29、关节速度 29、上次动作 29；合计 160 |
| 关节顺序 | 训练时 PhysX 顺序；导出脚本按名称重排 `.npz` 关节，不按文件原列顺序推断 |
| 归一化 | checkpoint 中的经验均值/方差，采用训练 MuJoCo 回放的 `(obs - mean) / sqrt(var + 1e-6)` |
| 目标角与 PD | 每关节 `default_q + action_scale * action`；`action_scale = 0.25 * effort_limit / kp`，`kp/kd/effort_limit` 与训练 `robots/x1.py` 对齐；并联踝按关节空间 PD 力矩映射到电机 |

策略输出每 20 个控制周期保持一次，当前未额外叠加 LPF，以维持与训练/回放相同的动作语义；实机若发现高频抖动，应先定位状态估计、关节速度和执行器闭环问题，再通过闭环测试评估是否需要滤波及其延迟。

模型和动作由 `motion_control/module/control_module/tools/export_beyondmimic_x1.py` 导出。当前资产：

| 部署文件 | SHA256 |
| --- | --- |
| `motion_control/module/control_module/policy/dance_x1_4999.bin` | `6939c1c69a6d509a85b4f7cfbbbd0ea8dae875f7507a30256364f3e3ec1c5f77` |
| `motion_control/module/control_module/policy/dance_x1_motion.bin` | `dd3376f615ee86daf6df4f1b13972942376d5a2275e904c4efb1674691bf884d` |

## 必需的外部状态估计输入

训练策略使用了仿真中直接可得的腰部锚点世界位姿和机身线速度。当前 DCU 仅提供 IMU 与关节反馈；**不能把缺失观测填零或用姿态角代替位移/速度**。在真机上，必须先由经过验证的状态估计与正运动学链路提供：

| 话题 | ROS 2 类型 | 坐标与含义 |
| --- | --- | --- |
| `/dance/anchor_pose` | `geometry_msgs/msg/TransformStamped` | `header.frame_id=odom`，`child_frame_id=lumbar_pitch_link`；腰部锚点在连续 odom 世界系中的真实位置与朝向 |
| `/dance/base_velocity` | `geometry_msgs/msg/TwistStamped` | `header.frame_id=base_link`；`twist.linear` 为机身坐标系线速度，单位 m/s |
| `/imu/data` | `sensor_msgs/msg/Imu` | 机身 body→world 四元数与机身坐标系角速度 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 29 个关节名称、实际角度与角速度；与 `joint_offset` 标零规则一致 |

舞蹈控制器要求上述反馈在 80 ms 内持续更新、数值有限、话题帧名正确；进入前还检查速度、倾角和关节机械限位。**控制器按接收时间判断新鲜度，无法单独识别上游反复重发的旧传感器样本。**状态估计器应自行校验源时间戳和失联，并在失效时停止发布。现场还必须确认 odom 的高度基准、锚点 FK 与 IMU 坐标定义。

`scripts/dance_state_bridge.py` 可在 Linux ROS 2 主控上把 `/odom` 与 TF 链转换成上述两个舞蹈话题。它要求 `/odom.header.frame_id=odom`、`/odom.child_frame_id=base_link`，且 `twist.linear` 确实在 `base_link` 坐标系；TF 必须连续提供 `odom → lumbar_pitch_link`。脚本会检查原始时间戳、新鲜度和两个源的时间差，条件不满足就停止发布。部署前仍需在实机确认 TF 的动态关节链、线速度坐标、地面高度和估计质量；脚本不会凭空生成缺失的状态估计。

进入时，控制器用实测锚点位姿对齐参考动作第 0 帧的世界位置和朝向，再用 4 秒三次曲线从当前关节角过渡到首帧姿态。参考动作的首帧手臂姿态与普通站姿差距较大，过渡必须先在吊架内验证。准备末端若未跟上目标角，或运行中观测超时、倾角、关节/锚点误差越界、模型输出异常，会退出到 `keep`；硬件急停仍需独立可用。

## Linux 主控验证顺序

1. 先按上一节核实腰俯仰方向、零位、限位和锚点 TF。当前 `sign_calibration_verified: false` 时主控可以启动以检查反馈，但 `/dance_mode` 会被拒绝。保存现场核实记录后才开启该开关。
2. 在服务器上获取**父仓库与 `motion_control` 子模块同名分支的代码**，更新父仓库子模块指针；同时确认两个 `.bin` 资产已包含。本地提交尚未推送时，服务器无法通过 Git 远端获取这些改动。
3. 在 Linux 主控运行 `./scripts/build.sh`。检查 `build/cfg/x1_cfg_dance.yaml`、`build/run_dance.sh` 和两个模型资产；用 `sha256sum` 与上表比较。每次重新生成 `build/aimrt_main` 后，在 `build/` 内执行 `sudo setcap cap_net_raw,cap_sys_nice=ep ./aimrt_main` 并用 `getcap` 检查，避免启动脚本仅设置 `cap_net_raw` 时失去实时调度权限。不要在 Mac 上编译或安装。
4. 吊装机器人，确认硬件急停和驱动器失联下使能。先将 `build/cfg/dcu_driver_module/dcu_x1.yaml` 设为 `enable_actuator: false`，校验 EtherCAT、29 关节、IMU、两个舞蹈状态话题和实时性。此阶段直接从 `build/` 运行 `./aimrt_main --cfg_file_path=./cfg/x1_cfg_dance.yaml`；头部需另行支撑，避免启动脚本使能颈部舵机。
5. 先检查 `/odom` 与 TF 的帧名、方向、时间戳和频率；若满足上述契约，可在 Linux ROS 2 环境运行 `python3 scripts/dance_state_bridge.py`。确认桥接后的锚点姿态、线速度与实测一致，特别检查静止、缓慢平移、缓慢转向及下蹲时的方向和高度。无真实状态估计输入时，`/dance_mode` 应被拒绝。
6. 完成独立的同控制器闭环仿真、硬件限位与力矩检查后，才在吊架中将 `enable_actuator` 改为 `true`。从 `build/` 执行 `bash ./run_dance.sh`；该脚本可能使能颈部舵机，应核对串口和头部保持状态。依次使用 `/zero_mode`、`/stand_mode`、`/dance_mode`，每步等待状态日志及机械稳定，普通模式间隔至少 1 秒。舞蹈期间保持急停可触及；策略完成后应自动进入 `keep`。
7. 留存 `build/log/x1_rl_ctrl_dance.log`、控制周期统计、关节与执行器命令/反馈，检查 4 秒姿态过渡、50 Hz 推理时序、并联踝力矩、关节限位、锚点误差及故障退场。任何异常立即使用硬件急停。

现有训练仓库的 MuJoCo 记录显示 5 次循环（3235 策略拍）未检测到跌倒，但该回放在每次循环时把机器人重置到参考首帧。真机不能瞬移，本分支采取单次播放和过渡启动；因此训练回放结果**不能替代本分支的闭环仿真和吊架验证**。
