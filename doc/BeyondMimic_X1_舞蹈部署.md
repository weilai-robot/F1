# X1 BeyondMimic v11 舞蹈部署审核与使用说明

2026-10-17。本版使用 `Beyondminic-Weilai-X1/trained_models/dance_x1_v11_model_4999.pt`，在 `motion_control` 的 AMP 部署基线 `ec29ebe` 上补入并修复舞蹈部署。主仓库分支为 `feat/x1-beyondminic-dance-deploy`，子模块的 v11 专用分支为 `feat/x1-dance-v11-deploy`；主仓库记录包含源码、配置与模型资产的子模块提交。既有 AMP 分支保持原提交。

**Mac 已完成离线模型核对、Python 数值仿真及不依赖 ROS 的 C++ 就绪门控检查。2026-10-22 已补 IMU/双腿本体估计器，并完成实际 C++ 核心的离线测试和估计反馈闭环。用户已在现场试跳，反馈多次演示不一致；完整 Ubuntu C++/ROS 2 构建、完整 C++ 单测及系统验收结果尚未在本记录中核验。现有结果不能作为真机部署通过的结论。**

## 当前按键流程

使用专用舞蹈手柄配置 `joy_x1_dance.yaml`。正常启动只按三次：**A → B → Guide**。A 到零位后等待 B，B 到 stand 后等待 Guide；一次 Guide 自动执行准备和播放，不需要 X 或再次按 Guide：

| 按键 | 状态/话题 | 行为 |
| --- | --- | --- |
| A | `zero` / `/zero_mode` | 从实测姿态用 2 秒过渡到零位并保持；到位提示 `Zero reached; press B for stand` |
| B | `stand` / `/stand_mode` | 零位到位后，2 秒过渡到 AMP stand 并保持；提示 `Stand reached; press Guide to prepare and dance` |
| Guide | `dance_ready → dance_x1` / `/dance_start` | 4 秒过渡到 `default_q`，连续满足就绪条件 4 秒后，自动从参考帧 0 开始 50 Hz 播放 |
| Back / Start | `keep` / `idle` | 零位、stand、ready 及播放阶段均可立即退出 |

A、B、Guide 都是按下沿触发。提前按 B 或 Guide 会被拒绝，需松开重按；站稳后已接受的 Guide 则授权整段准备和自动播放。重复 Guide 不会重置准备或重入播放。就绪等待使用单调时钟，姿态、速度或外部估计不合格时重新计时；控制检查间隔超过 80 ms 同样重新计时。零位/stand 的实测关节误差须 ≤0.12 rad，反馈缺失/过期或倾角超过 35° 时拒绝进入或切 keep。

零位是关节目标全零，不执行编码器校准。零位不会自动转 stand，stand 也不会自动开始准备；只有 Guide 后的准备到播放使用内部 `/dance_play` 完成事件，控制模块不订阅该话题。Back/Start 退出或按 A 返回零位后，自动播放请求即取消。stand 显式控制全部 29 关节，角度与增益等于 AMP 原 `pd_zero + pd_stand` 的覆盖结果。

**当前准备位已选择训练 `default_q`：腰三轴、双臂和腕部归零，双腿适度弯曲。** `dance_x1.start_pose: default_q` 与 `pd_dance_pose.init_state` 一起校验；Guide 到位检查也使用这一姿态。策略的参考时钟仍从动作第 0 帧开始，不跳帧、不瞬移。

该默认角是训练的动作偏置与名义关节配置，不能等同于训练 episode reset 的完整状态。训练 reset 从参考中采样，并加入关节/根状态扰动及速度；真机静止准备不具备参考首帧的非零初速度。

准备位候选比较与结果见 `doc/analysis/x1_dance_v11_20261017/preparation_recommendation.md`。`default_q` 在 1–30 秒保持测试中均未触发准备阶段保护，双脚持续接触；此前 6 组手动等待 configured 探针完成整段播放，但较短保持曾保护退出。当前自动接管按实际 `dance_ready_hold_s` 连续等待，就绪不合格会延长准备，不能用历史手动保持结果替代新流程验证。

当前等待设为 **4 秒连续就绪**：最初 3 秒自动接管在 yaw=-1.2 时第 153 拍触发锚点保护；4 秒候选在 yaw=0/0.8/-1.2 三组均完成 647 拍，正式 YAML 的 yaw=-1.2 复核也完成 647 拍。当时保留原 0.6 m 锚点退出阈值，后续根据现场反馈已移除该退出判断。失败原始记录保留；这些有限样本不能证明任意初始状态都能接管。

## 对 AMP 已通过链路的复用

| 项目 | 舞蹈部署行为 |
| --- | --- |
| 主循环 | 复用 `ControlModule` 的 1000 Hz 主循环、实时调度、状态机和关节命令发布 |
| 策略时序 | 与 AMP 一致，每 20 拍推理一次，50 Hz；关节目标在两次推理间 ZOH 保持 |
| 接管 | stand 按 Guide 进入 `dance_ready`；过渡及就绪等待完成后自动推理；策略内部 `prepare_ticks: 0` |
| PD 执行 | 普通关节下发位置和 kp/kd；踝关节每 1 ms 用最新 q/qd 算软件 PD，关节力矩限制 ±80 Nm |
| LPF | 不叠加动作或踝力矩 LPF |
| DCU | 继续使用当前 AMP 的传动方向、MIT 饱和处理、命令同步及有效反馈处理 |
| AMP 配置 | `rl_x1.yaml`、`rl_x1_sim.yaml`、AMP 模型、`rl_controller.cc` 和 DCU 文件本次均未修改 |
| 专用入口 | `run_dance.sh` → `x1_cfg_dance.yaml` → `rl_x1_dance.yaml`，加载专用舞蹈手柄配置，不加载 AMP 策略 |

舞蹈与 AMP 的**观测和动作语义不同**：舞蹈是 160 维单帧观测、29 维动作，动作缩放按每关节 `0.25 * effort_limit / kp`；AMP 的 288 维历史输入和统一 `0.25` 缩放不能直接移植。

## 审核发现与修复

1. 旧导出器以早期 armature 和固定频率公式重算 kp/kd，与 v11 训练配置不一致。现在通过 AST 读取训练仓库 `robots/x1.py` 的名义 kp、kd、armature、effort 和默认角；不导入或执行 Isaac Sim。同步重算动作缩放，并保存逐关节参数及来源 SHA256。armature 是仿真动力学参数，不额外转成真机补偿力矩。
2. 旧资产未使用 v11 权重。新资产命名为 `dance_x1_v11_4999.bin`，格式升级为 `X1DNCE2`，控制器会拒绝旧格式。新格式同时保存辨识惯量，便于仿真和来源核对。
3. 训练仓库的 `play_mujoco.py` 使用 `sqrt(var + 1e-6)`，与 RSL-RL 默认推理的 `_std + 0.01` 不同。部署现在使用 checkpoint 保存的 `_mean`、`_std` 和后者公式。该默认值及计算位置可见 [RSL-RL v2.3.3 EmpiricalNormalization](https://github.com/leggedrobotics/rsl_rl/blob/v2.3.3/rsl_rl/modules/normalizer.py)。训练云镜像没有随 checkpoint 保存依赖锁；Ubuntu 验收时仍应核对实际训练版本有无自定义 eps。没有修改训练仓库。
4. 旧舞蹈配置额外对腰俯仰乘 -1。当前 AMP 基线已在 DCU 提交 `fb8cc78` 根据实测修正 `direction_right: -1`，`5c13694` 将模型关节限位统一为 `[-0.15, +0.45]`。舞蹈现在 29 个关节均直接使用模型坐标，不再二次反号；TF/FK 同样应直接使用修正后的 `/joint_states`。
5. A 零位、B stand、Guide 启动准备并自动播放。准备阶段使用高增益 PD 做 4 秒过渡和连续就绪等待，不运行策略、不推进参考帧。内部切播放时在组装关节命令前重启策略并推理首帧，避免混合准备/播放命令。准备目标与模型 start_pose 逐关节核对，配置过期时拒绝启动。
6. 根据现场实测反馈，已移除播放阶段的 `dance joint tracking error or limit exceeded` 和 `dance anchor tracking error exceeded limit` 看门狗：实测关节越限、瞬时 PD 跟踪误差及锚点位置偏差不再使播放退出。`max_tracking_error`、`max_anchor_error` 配置已移除，旧 YAML 中同名字段不再生效。锚点位置偏差仍按原定义输入 policy，不重新对齐或清零。关节目标仍严格裁到全局限位，踝关节力矩仍限幅；启动前继续检查实测关节限位，允许 0.01 rad 容差。倾角、反馈超时、非有限输出和动作过大仍会退出；Back/Start 的 keep/idle 遥控退出路径保留。
7. 参考只在播放开始时对齐世界位置与 yaw，不以完整 3D 旋转消除真实躯干 pitch/roll 误差。单次播放完成正常切到 `keep`，不再记为模型故障或循环重置。
8. 旧 `run_dance.sh` 设置的配置变量被 `run_real_nav.sh` 忽略，实际会启动导航配置。现在正确传递配置路径；权限检查和设置同时保留 `cap_net_raw`、`cap_sys_nice`。故障/完成切入 keep 时当拍发布新捕获的保持指令。
9. 外部反馈同时检查源时间戳、递增性和接收超时；旧样本重发不会续命。默认由 C++ 直接接收完整身体里程计，按其源时间插值腰部关节并计算锚点，无需独立 Python 或腰部 TF。准备阶段每 2 秒报告当前等待原因。脚本 `dance`/`dance-start` 等价于 Guide，等待 DDS 匹配后只发一次、使用 volatile durability，不重试提前请求。移除旧 `dance-ready` 和 `/dance_mode` 入口。相关 CLI 参数依据 [ROS 2 Humble ros2topic 实现](https://github.com/ros2/ros2cli/blob/humble/ros2topic/ros2topic/verb/pub.py)。既有 AMP 手柄配置与导航 `ready` 入口保持原意。

## 模型与参数

| 项目 | 值 |
| --- | --- |
| Actor | 160 → 512 → 256 → 128 → 29，ELU，确定性均值输出 |
| 参考 | `dance_x1_train.npz`，647 帧、50 Hz、单次 12.94 秒 |
| 观测顺序 | 参考 q29、参考 qd29、锚点位置误差3、相对旋转前两列按行展开6、body 线速度3、body 角速度3、q-default29、qd29、上次 action29 |
| 关节顺序 | 已核实的 PhysX 顺序；参考 NPZ 按关节名称重排 |
| 归一化 | `(obs - _mean) / (_std + 0.01)` |
| 目标 | `clip(default_q + action_scale * action, lower, upper)`；不加额外 0.03 rad 裁边 |
| 力矩/限速 | 训练 effort 沿用 v11；硬件仍由当前 AMP/DCU 驱动执行，腕部 L28 的内部位置环并不等价于训练 PD |

| 关节组 | kp / kd | armature |
| --- | --- | --- |
| hip pitch | 65 / 8 | 0.208 |
| hip roll | 60 / 8 | 0.25 |
| hip yaw | 100 / 3.3 | 0.0148 |
| knee pitch | 85 / 8 | 0.273 |
| ankle pitch / roll | 50 / 4；50 / 2.5 | 0.15；0.035 |
| lumbar yaw / roll / pitch | 90 / 8；70 / 8；100 / 8 | 0.022；0.021；0.012 |
| shoulder pitch / roll / yaw | 40 / 4；40 / 4；40 / 2 | 0.02；0.013；0.01 |
| elbow pitch / yaw | 30 / 1.9；20 / 1 | 0.01 |
| wrist pitch / roll | 15 / 1 | 0.01 |

| 部署文件 | SHA256 |
| --- | --- |
| `policy/dance_x1_v11_4999.bin` | `4b146822e4e37dfc1c73e35b5712d8893aacfcd82d5fb82e59d47f66b070d239` |
| `policy/dance_x1_motion.bin` | `dd3376f615ee86daf6df4f1b13972942376d5a2275e904c4efb1674691bf884d` |

`policy/dance_x1_v11_4999.json` 保存 checkpoint、训练配置、参考动作的来源 hash 和所有逐关节参数。导出时 129 组输入的序列化模型与独立 PyTorch 推理比较，最大动作绝对误差为 `2.29e-5`。

2026-10-22 并行进展：用户正在训练 154 维观测模型。本地 `WoStateEstimation` 配置从 actor 去掉锚点位置误差 3 维和机身线速度 3 维，仍保留相对姿态等输入；实际训练配置及结果待核验。当前导出器、控制器和诊断仍固定为 160 维，154 维模型不能直接替换现有资产。输入精度改进路线、两种模型的观测索引和后续适配/对照计划见[输入精度与 154 维模型计划](analysis/x1_dance_v11_state_estimator_20261022/INPUT_ACCURACY_PLAN.zh_CN.md)。

重新导出可在 Mac 已有 Python 环境离线进行：

```bash
python motion_control/module/control_module/tools/export_beyondmimic_x1.py \
  --training-root /path/to/Beyondminic-Weilai-X1 \
  --checkpoint /path/to/Beyondminic-Weilai-X1/trained_models/dance_x1_v11_model_4999.pt \
  --motion /path/to/Beyondminic-Weilai-X1/whole_body_tracking/motions/dance_x1_train.npz \
  --model-out motion_control/module/control_module/policy/dance_x1_v11_4999.bin \
  --motion-out motion_control/module/control_module/policy/dance_x1_motion.bin
```

## 当前流程的离线验证

- 状态图、A/B/Guide 按下沿映射、移除 X、内部播放事件不订阅、29 关节目标/限位、default_q 资产一致性及 SHA256、YAML/脚本检查通过。stand 角度/增益与 AMP 组合结果一致。
- Mac 已编译运行实际 `DanceStartGate` 与 `ButtonTrigger` 头文件组成的独立检查：只产生一个 Guide 事件，连续 4 秒后就绪、不需要第二次按键，反馈不合格/超过 80 ms 重计时及退出复位均通过。脚本 Guide 使用 mock 命令验证为单次 volatile 事件，没有运行 ROS 或电机。
- 当前自动接管探针记录在 `guide_auto_start/`，包含 3 秒等待的失败记录和 4 秒候选的三朝向通过记录。物理仿真从已站立状态开始，不涵盖冷启动零位和 B 进入 stand 的实际物理过程。旧 `three_button_flow/` 是历史 A→X→Guide 流程，已标记为历史记录。
- 已补 Ubuntu C++ 回归测试：三键零位/stand/Guide 准备、准备后内部自动播放、重复 Guide 不重置、退出取消自动播放、全阶段急停和 29 关节 AMP 一致性。这组依赖 ROS/Eigen/gtest 的完整 C++ 单测未在 Mac 执行。
- 关节看门狗移除后，新增 C++ 回归覆盖旧 `max_tracking_error` 字段不生效、播放中上下限越限继续推理且目标仍限幅、越限姿态仍不能启动；待 Ubuntu 执行。Mac 使用 NumPy float32 MLP/ELU 和生产 C++ 估计核心运行 MuJoCo 离线探针：正常播放及单拍注入左右髋滚转上下限外 0.02 rad 反馈均完成 647 帧；这不等于完整 C++ 控制器或真机验收。
- 锚点看门狗移除后，新增 C++ 回归覆盖旧 `max_anchor_error` 字段不生效、锚点位置偏移 0.65 m 后仍继续推理、偏差仍影响 policy 输出且目标仍限幅；待 Ubuntu 执行。Mac 的 NumPy/MuJoCo 离线探针中，正常播放和在一次 actor 推理时注入世界 x 方向 0.65 m 偏移均完成 647 帧；注入场景的最大 policy 锚点误差为 0.6603 m，原退出判断对应的误差最大为 0.6457 m。该探针包含生产 C++ 估计核心，未运行完整 ROS/C++ 控制器。
- C++ 原生状态接入的核心检查已在 Mac 用 Clang + AddressSanitizer/UndefinedBehaviorSanitizer 编译运行：腰部 FK 与训练 MJCF 的 70 组独立 MuJoCo `mj_forward` 结果一致，包含非零腰三轴及任意机身旋转；源时间插值、环形缓冲覆盖、缺样本、过大间隔和重复时间戳拒绝检查通过。独立期望值记录在 `motion_control/module/control_module/test/data/dance_x1_waist_fk.csv`，包含训练 XML SHA256。另补原生里程计/旧外部状态推理一致性、错误帧、同步等待、过期/无效输入及估计器冻结的 ROS 控制器测试，尚未在 Ubuntu 执行。核心检查不等于状态估计器或真机通过。
- 先前的参考首帧准备位探针：stand 驻留 3 秒 → 高增益 4 秒准备 → 计划保持 1 秒 → Guide；准备开始后 4.451 秒触发 35° 倾角保护，`policy_steps=0`。此准备位已被 default_q 取代，历史报告为 `sim_stand_pose_guide.json`。
- 最初的 default_q 诊断只检验保持，历史报告为 `sim_default_q_preparation_probe.json`。随后比较了 9 种候选与 18 组驻留/朝向组合，选择 default_q。旧手动保持配置的 6 组探针（3/5/10/30 秒、若干 yaw 组合）均完成 647 拍，记录在 `preparation_comparison/configured_*.json`，不作为当前自动接管的通过记录。

## 先前直接接管的离线记录（不作为当前流程验收）

Mac、MuJoCo 3.14.0；没有在 Mac 编译或运行 Ubuntu/ROS 2 部署程序。

- 复现原训练回放的 3235 拍结果：身体位置误差 0.1270 m、关节误差 0.2806 rad、无跌倒。改用 RSL-RL 归一化后：0.1178 m、0.2820 rad、无跌倒。这两组回放会在每轮重置机器人，不能验证实际接管。
- 下表是先前从 AMP 浅蹲直接接管的结果；当前入口是 A 零位 → B stand → Guide（准备 → 自动播放）。下表不作为新流程通过证据。`tools/sim_beyondmimic_x1.py` 已按自动接管更新。

| 场景 | 结果 | 最低 base z | 平均锚点误差 |
| --- | --- | --- | --- |
| 站姿驻留 3 秒，yaw=0 | 完成 647 拍 / 12940 个物理步，最大倾角 28.51° | 0.518 m | 0.165 m |
| 驻留 1 秒，yaw=0.8 | 完成 647 拍，最大倾角 28.17° | 0.515 m | 0.134 m |
| 驻留 5 秒，yaw=-1.2 | 第 187 拍锚点偏差超过 0.6 m，探针按部署阈值中止 | 0.580 m | 0.330 m |

第三项暴露了起始状态下的跟踪敏感性，本次保留 0.6 m 退出阈值，没有为了通过探针而放宽。**不能据此宣称任意站姿/驻留时间下的舞蹈接管均已通过。**原始报告在 `doc/analysis/x1_dance_v11_20261017/`。

Python 探针使用训练 MJCF 和理想状态反馈，并以 effort 饱和的关节 PD 模拟全部执行器；它没有运行 C++ 控制器，也未覆盖真实估计延迟、网络抖动、L28 腕部内部位置环或 EtherCAT/MIT 编码行为。已补充 Ubuntu C++ 单测，覆盖 v11 首帧 golden 输出、20 拍目标保持/踝力矩更新、缺失/旧/重复反馈，以及频率、静持阶段、关节顺序契约；这些 C++ 测试尚未运行。

## Ubuntu 上的构建与接入

服务器更新主仓库后，需要同时检出主仓库记录的子模块提交：

```bash
git fetch origin
git switch feat/x1-beyondminic-dance-deploy
git pull --ff-only
git submodule update --init --recursive motion_control
```

确认服务器本地修改已保存后再更新；子模块检出到固定提交的 detached HEAD 属正常情况。部署不需要 `build_simtest/` 或 Mac 虚拟环境。

在 Ubuntu、ROS 2 环境运行；Mac 不安装 Linux 驱动或构建依赖：

```bash
./scripts/build.sh -DXYBER_X1_INFER_BUILD_TESTS=ON
ctest --test-dir build --output-on-failure -R '(control_module|joy_stick_module)'
# 测试通过后关闭 coverage，再生成真机运行版本：
./scripts/build.sh Release -DXYBER_X1_INFER_BUILD_TESTS=OFF
```

构建脚本核对专用配置、启动脚本和两份二进制资产及 JSON。当前 `x1_cfg_dance.yaml` 使用 1000 Hz 实时控制线程，复用 DCU 配置，初始模式为 `idle`。先用 `enable_actuator: false` 和直接执行 `aimrt_main` 检查反馈，避免启动脚本使能颈部舵机；执行器使能与吊架验证沿用既有 AMP 流程。

舞蹈比 AMP 额外需要身体位置和线速度。当前默认由内置本体估计器补齐，输入与输出如下：

默认启用独立的 50 Hz 舞蹈诊断：`build/test_logs/data_csv/dance_diag_<Unix纳秒>.csv`，并保存对应 `.csv.yaml` 配置快照。按演示编号/动作帧记录 policy 实际使用的归一化前后输入、估计偏置/方差、足端接触/载荷、关节反馈、候选 action 和生成的关节命令；准备阶段也采样，独立线程默认每 100 ms flush。详细字段、采样边界和多次演示对比方法见 [舞蹈诊断日志](analysis/x1_dance_v11_state_estimator_20261022/DANCE_DIAGNOSTICS.zh_CN.md)。旧 `ctrl_period_*.csv` 继续用于主线程时序分析。

| 话题 | 类型 | 契约 |
| --- | --- | --- |
| `/dance/odometry` | `nav_msgs/msg/Odometry` | 默认是 50 Hz 检查输出；完整 `odom → base_link` 位姿，twist 在 base_link，均对应机身原点；内部反馈随传感器更新 |
| `/imu/data` | `sensor_msgs/msg/Imu` | 输入 body→world 四元数、body 角速度和含重力的比力，单位 m/s²；静止模长应约为 9.81 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 输入 29 关节模型坐标位置/速度/力矩；本体估计使用双腿 12 关节，沿用 AMP 的 offset 和 DCU 方向 |

真机这三种反馈必须有有效、递增的系统时钟源时间戳且在 80 ms 内更新；错误帧名、重复旧样本或断流都会使播放进入被拒绝或运行退出。`use_sim_handles: true` 仅离线仿真使用，跳过系统时钟年龄比较，仍检查源时间递增和接收超时。

默认 `controllers.dance_x1.state_source: proprioceptive`：IMU 源时间与双腿位置/速度/力矩按 ≤10 ms 间隔插值配对；0.5 秒连续静止且脚承重后完成加速度偏置校准。用 IMU 预测、足端位置/零速度/平地高度约束更新卡尔曼滤波，保留 IMU 的完整姿态与 yaw。双腿几何和质量从 v11 训练 MJCF 导出；准静态关节载荷估计与足端速度一致性共同判断接触。该接触方法不是官方 G1 的完整广义动量观测器。默认 IMU 平移偏置为训练值 `[0,0,0]`，需核实实机下肢 DCU 的安装位置和轴向，必要时配置 `state_estimator.imu_position_in_base`。完全悬空或加速度单位错误时不会完成校准。超过 10 ms 的源时间缺口重新校准，可信接触丢失超过 0.35 秒使估计失效。仅依赖本体信息，无法独立校正持续滑移或 yaw 漂移。

估计结果直接交给既有里程计/腰部 FK 路径，不经 DDS 回环。`DanceController` 使用训练模型的 base_link → lumbar_yaw → lumbar_roll → lumbar_pitch 三轴变换，按身体状态源时间查找腰部关节反馈，既有 80 ms 时效检查保留；FK 不再二次反号。锚点位置偏差仍输入 policy，但播放阶段不再因偏差超过 0.6 m 退出。2026-10-22 首次验证的三种 yaw 估计反馈均完成 647 帧，播放期间位置 RMSE 约 1.8–2.1 cm、速度约 0.013 m/s；一组显式合成噪声探针也完成。实际生产估计核心参与仿真，但没有运行完整 ROS 控制器或 DCU。详见 [本体估计验证记录](analysis/x1_dance_v11_state_estimator_20261022/README.md)。

仍可显式选择 `state_source: odometry`，由 `ControlModule` 订阅 `sub_dance_odometry_name`，默认 `/dance/odometry`；此模式不启用内置估计器。外部消息必须是完整身体状态且 twist 在 base_link，同 IMU 使用一致的世界 yaw 和源时钟。现有导航 `/odom` 使用 `base_footprint`、压平姿态并重写时间戳，且当前 FastLIO2 发布没有填写 twist，不能直接改名代用。自定义话题名还需同步修改顶层 ROS 订阅匹配规则。

`/dance/odometry` 的 ROS 后端订阅显式使用 best_effort、volatile、keep_last/depth=5，支持传感器数据发布端；自定义话题名时也要同步修改这一 QoS 规则。字段位置依据 [AimRT ROS2 插件配置](https://docs.aimrt.org/v0.8.3/tutorials/plugins/ros2_plugin.html)。是否完成 DDS 匹配仍需在 Ubuntu 用 `ros2 topic info -v` 验证。

保留兼容模式 `state_source: external_topics`：此时仍订阅 `/dance/anchor_pose` 和 `/dance/base_velocity`，可使用 `scripts/dance_state_bridge.py`。仅此模式需要外部同源时间的腰部 TF；默认原生路径不运行该脚本。

Ubuntu 完成构建、准备位确认、状态估计接入和吊架验证后，启动和模式入口为：

```bash
cd build
bash ./run_dance.sh
```

另一终端在仓库根目录、同 ROS domain 下：

```bash
# 默认由内置估计器生成 /dance/odometry；脚承重静止校准后再播放，对应 A → B → Guide。
bash scripts/switch_x1_mode.sh zero        # A：零位并保持
# 等待 Zero reached; press B for stand
bash scripts/switch_x1_mode.sh stand       # B：stand 并保持
# 等待 Stand reached; press Guide to prepare and dance
bash scripts/switch_x1_mode.sh dance-start # Guide：准备位 → 自动跳舞，无需再按键
# 舞蹈单次结束自动 keep；需要手动保持时：
bash scripts/switch_x1_mode.sh keep
```

脚本 `ready` 是原有导航的 zero → stand → walk，不用于舞蹈。舞蹈使用 `zero`、`stand`、`dance-start`；`dance` 是 `dance-start` 的别名，旧 `dance-ready` 已移除。入口禁止重入清状态；`keep/idle` 在整段流程退出均不受普通模式 1 秒节流阻塞。保留 `build/log/x1_rl_ctrl_dance.log` 和控制周期记录，核对零位/stand 到位、Guide 后自动接管、50 Hz 推理、锚点误差和故障切换。仍需 Ubuntu 原生测试及真机吊架验收。

## Guide 后下沉但不播放的排查

轻微下沉可能是已经进入 `dance_ready` 的 PD 准备位，不能据此判断策略已运行。控制日志应先出现 `Guide accepted`，连续就绪后才出现 `Ready hold complete; starting dance automatically from frame 0`。过渡完成后，真实关节姿态、IMU、锚点和速度须连续满足条件 4 秒；任一条件失效都会重计时。当前 C++ 控制日志每 2 秒输出一次 `Dance preparation: ...`，区分准备位过渡、无身体里程计、帧名/时间戳错误、腰部样本未覆盖源时间、速度/倾角和准备位误差等原因。

机器人上另开终端，在 F1 根目录执行以下只读检查，无需再次按 Guide：

```bash
source scripts/ros2_source.sh
tail -n 60 build/log/x1_rl_ctrl_dance.log
timeout 6s ros2 topic echo /dance/odometry --once
timeout 6s ros2 topic hz /dance/odometry
ros2 topic info -v /dance/odometry
```

- 没有 `Guide accepted`：检查是否已在 stand 到位，或准备阶段是否已保护退出。
- `/dance/odometry` 无持续数据：默认模式检查脚是否承重且静止、双腿力矩是否有效、加速度是否为 SI 比力、源时间是否连续；控制器会报告校准/等待原因。外部 `odometry` 模式则检查其发布节点。
- `waist joint samples do not bracket ...`：核对估计器是否使用真实采样时间、与关节反馈是否同一系统时钟，以及关节反馈间隔是否足以覆盖源时间；不要将源时间改成发送时刻。
- 有里程计仍不播放：检查 `Dance preparation` 原因、源时间递增和 QoS/订阅匹配，以及 29 关节准备位误差 ≤0.12 rad、关节限位、身体线速度 ≤0.2 m/s、倾角 ≤35° 及三种反馈年龄 ≤80 ms。话题频率正常不代表这些条件已满足。
- 仅在兼容模式排查旧桥：两个 `/dance/*` 无数据时看 `Not publishing dance state: ...`，再检查 `/odom` 和同源时间腰部 TF。默认原生路径无需检查这两个旧话题或运行 Python 桥。

控制器是否收到有效数据、是否连续就绪以控制侧检查为准。不要通过将 `use_sim_handles` 改为 `true`、放宽反馈超时或伪造状态让真机进入播放。
