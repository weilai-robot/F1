# AMP部署操作说明

适用范围：`feat/amp-deploy` 分支、X1 29DOF、`x1_amp_soup59c_50hz.onnx`，部署目标为连接 X1 的 Linux 主控。以下命令均应在 **Linux 主控**执行。本说明只用于受保护的吊架/悬空联调；完成这些步骤不代表可以无保护地面行走。训练和模型对齐结果见 [AMP 部署审计](amp_deploy.zh_CN.md)。

## 腰部方向测试结果与修正

现场已确认：在原 `direction_right=+1` 下，B 的腰 pitch +0.10 rad（约 5.7°）相比 A 的 0 让腰部向后折，与模型正角度向前折的约定相反。

已将 `dcu_x1.yaml` 中 **`lumbar_parallel_trans.direction_right` 改为 -1**，由传动层同时转换腰 pitch 的位置、速度、力矩命令和反馈。左右脚踝等其他 `direction_right` 保持原值。真机和仿真的 zero 腰测试目标已恢复 0，stand 腰目标仍为 0，AMP 模型及 kp/kd 保持原配置。

下文构建安装后须重启控制进程，运行配置 `build/cfg/dcu_driver_module/dcu_x1.yaml` 的 `lumbar_parallel_trans` 应显示 `direction_right: -1.0`。先在有支撑时复测 B、A；新方向下 AMP 的动作与反馈都已改变，仍需真机复测。

按用户要求，真机及部署仿真的腰 pitch 目标限位现已同步训练 URDF：**`[-0.15,+0.45]` rad（后折 −8.59°，前折 +25.78°）**。先前 `+0.15` rad 上限已更新；右腰传动方向保持 -1，zero/stand 腰目标保持 0。此处配置是关节坐标中的目标裁剪范围，实际机械行程仍由现场复测确认。若现场有自定义标零偏移，改变方向后需按关节新坐标重新核对；仓库默认 pitch offset 为 0。

## 默认手柄固定 1 m/s 测试

本分支已将真机 `rl_x1.yaml` 配置为 `initial_state: idle`、AMP `constant_cmd: [1.0, 0.0, 0.0]`，与仿真的固定指令一致。`run.sh` 自动设置 `cap_net_raw,cap_sys_nice`，权限设置失败时不会启动控制进程。

在连接机器人的 Linux 主控执行：

```bash
cd ~/Robot_github/F1
git switch feat/amp-deploy
git pull --ff-only origin feat/amp-deploy
git submodule update --init --recursive
bash scripts/build.sh
cd build
bash run.sh
```

构建会安装真机配置和脚本，无需再手工添加固定指令或修改 setcap。现场网卡、DCU/CAN ID、IMU 和零位标定仍须与本机匹配；重建前保存现场参数，重建后核对。

确认启动日志出现 `cmd source: CONSTANT` 和 `amp CONSTANT CMD = [1.00, 0.00, 0.00]`，反馈正常，再按以下顺序操作：

| 操作 | 结果 |
|---|---|
| 启动后等待至少 1 秒，按 B | idle → zero；腰 pitch 目标已恢复 0 |
| 等待至少 2 秒且复位完成，按 A | zero → stand；腰 pitch 回到 0 |
| 浅蹲站稳后等待至少 3 秒，单次按 Guide | stand → walk_amp，立即持续执行 [1,0,0] 指令 |

进入 AMP 后不用再按键，不需要 LB 或摇杆。固定指令由 AMP 控制器在每次推理中直接使用，不依赖手柄持续发布速度话题；1 m/s 是策略目标速度，不保证真实速度恰为 1 m/s。

固定模式下，松开 LB、摇杆归零、发布零速度及速度话题超时都不会停车。正常退出按 A 切 stand（普通模式 1 秒节流）；Back 可在 AMP 内立即切 keep；异常倾倒或冲击用硬件急停。反馈断流、倾角超限、推理故障仍会自动退出 AMP。测试期间避免 RT/LT。

本地只完成部署模型独立闭环验证，真机首次复测保持吊架防跌保护。

## 1. 同步完整分支

先确认提交中包含 `motion_control` 子模块里的控制代码、两个 `rl_x1*.yaml`、`module/control_module/policy/x1_amp_soup59c_50hz.onnx`，并在父仓库提交更新后的子模块指针。**仅拉取父仓库分支而未提交子模块改动或 ONNX 文件，服务器拿不到本次 AMP 版本。**也可以传输完整工作树，但仍需按第 2 节校验产物。

在干净的 Linux 检出中执行：

```bash
cd /path/to/F1
git checkout feat/amp-deploy
git pull --ff-only
git submodule update --init --recursive
git status --short
```

`git status --short` 应为空；若服务器有自己的配置修改，先确认其来源和内容，再继续。

## 2. 在 Linux 主控构建并检查模型

目标环境需具备本项目要求的 Ubuntu/ROS 2 Humble、CMake、编译依赖及 EtherCAT 设备。仓库的构建脚本会自动查找 ROS 2 环境。**不要在 Mac 上构建或安装。**

```bash
cd /path/to/F1
./scripts/build.sh
test -x build/aimrt_main
test -f build/run_real_nav.sh
test -f build/cfg/x1_cfg_real_nav.yaml
test -f build/cfg/control_module/policy/x1_amp_soup59c_50hz.onnx
sha256sum build/cfg/control_module/policy/x1_amp_soup59c_50hz.onnx
```

ONNX 的 SHA256 应为 `3e09b76f1e38085a2093abccc1e6f80c86751c9758f7f21d0f497163c99ebd00`。另检查 `build/cfg/control_module/rl_x1.yaml` 中**所有** `policy_file` 指向的模型都已安装；模块启动时会初始化所有配置的控制器。构建失败或文件/哈希不一致时停止，不要使用旧的 `build/` 产物。

## 3. 核对真机配置与保护

首次接入时先将机器人可靠吊装，确认硬件急停可用、周围无人进入运动范围，并由现场人员支撑可能松动的头部。确认控制进程或 EtherCAT 断开后的驱动器超时/下使能行为；软件 `/keep_mode` **不能代替硬件急停**。

在 `build/` 目录核对以下配置，改动后保留版本记录：

| 文件 | 必须核对的项目 |
|---|---|
| `cfg/dcu_driver_module/dcu_x1.yaml` | `ethercat.ifname`、`bind_cpu`、`rt_priority`、DCU 的 EtherCAT ID、各执行器的 CAN ID、`imu_dcu_name`；首次总线检查设 `enable_actuator: false` |
| `cfg/control_module/rl_x1.yaml` | 顶层 `initial_state: idle`、`control_frequecy: 1000`、`bind_cpu`/`rt_priority`、29 个关节名称和顺序、实机 `joint_offset` 与机械 `joint_limits` |
| 同文件的 `rl_walk_amp` | `decimation: 20`、`action_scale: 0.25`、`settle_ticks: 0`、默认 `constant_cmd: [1.0, 0.0, 0.0]`、AMP 的关节/默认角/PD 顺序与训练 `lab_dof_names` 一致、`policy_file` 为本次模型、`lpf_conf.position_enabled: false`、`lpf_conf.effort_enabled: false` |
| `cfg/x1_cfg_real_nav.yaml` | 加载 `ControlModule` 与 `DcuDriverModule`，不加载 `JoyStickModule`；现场没有其他节点写 `/cmd_vel_limiter` |
| `cfg/x1_cfg.yaml` 与 `cfg/joy_stick_module/joy_x1.yaml` | 手柄模式入口：加载 `JoyStickModule`；Guide 键(8) 映射 `/amp_mode`，`constant_velocity` 已注释；AMP 默认固定 1 m/s，手柄只负责模式切换。删除 AMP 的 `constant_cmd` 后摇杆速度才生效 |

`initial_state: idle` 是 `rl_x1.yaml` 的**顶层**字段；本分支已显式设置，避免启动状态依赖 YAML 中状态的排列顺序。`ethercat.bind_cpu` 与控制器 `bind_cpu` 必须是该 Linux 主控上有效且合适的独立 CPU。首次启动前核对机身 IMU 的 body→world 四元数定义、29 关节反馈的方向/零位，以及并联踝双电机的传动符号和限幅。配置里的全零 `joint_offset` 不能代替实机标零。

50 Hz 策略每 20 个 1 kHz 控制周期推理一次。此模型训练和 sim2sim 验证未启用动作 LPF，所以 AMP 默认关闭位置目标及踝力矩 LPF，切入即执行策略（无 1 秒姿态平滑），保留关节目标限位和踝关节力矩限幅。若需改变 LPF，须重新验证该配置的闭环行为，不要在首次真机联调时临时开启。

## 4. 执行器不使能时检查总线与反馈

确保 `build/cfg/dcu_driver_module/dcu_x1.yaml` 中 `enable_actuator: false`。先检查 EtherCAT 网卡、实时内核和 CPU 配置，再从 `build/` 启动控制进程。这里直接运行二进制，不会由 `run_real_nav.sh` 自动使能颈部舵机；头部须另行支撑。

```bash
cd /path/to/F1/build
ip -br link
cat /sys/kernel/realtime
sudo setcap cap_net_raw,cap_sys_nice=ep ./aimrt_main
getcap ./aimrt_main
./aimrt_main --cfg_file_path=./cfg/x1_cfg_real_nav.yaml
```

`/sys/kernel/realtime` 应为 `1`；若实时内核、网卡或能力配置不符，先停止启动流程。

在另一 Bash 终端读取话题：

```bash
cd /path/to/F1
source scripts/ros2_source.sh
ros2 topic echo /joint_states --once
ros2 topic echo /imu/data --once
ros2 topic hz /joint_states
ros2 topic hz /imu/data
ros2 topic info -v /cmd_vel_limiter
```

两条 `ros2 topic hz` 命令分别短时运行，用 `Ctrl+C` 结束当前测量后再执行下一条。核对 EtherCAT 从站状态与设备 ID，无持续 WKC/DC 错误；日志中不得有实时调度或绑核失败。`/joint_states` 应持续返回 29 个唯一且与配置对应的关节，位置、速度、关节方向与实机一致。静止直立时，依据 `/imu/data` 的四元数计算的机身投影重力应接近 `(0, 0, -1)`；缓慢绕各轴倾斜，方向应与机身坐标系一致。检查反馈是否持续更新，避免 DCU 重发旧数据却仍形成新 ROS 消息。检查完用 `Ctrl+C` 退出进程。任一项不符，保持执行器不使能并先修正配置或硬件。

## 5. 外部速度模式：零速及低速测试

本节使用外部速度指令。必须先从真机 AMP 配置删除固定指令，再启动/重启进程；否则下面的零速度和 0.05 m/s 话题都不会生效：

```bash
cd ~/code/F1
sed -i '/^[[:space:]]*constant_cmd:/d' build/cfg/control_module/rl_x1.yaml
```

下次构建会恢复默认固定 1 m/s。默认手柄测试直接使用文档开头的流程。


仅在第 4 节全部通过、硬件急停及断线保护经现场验证后，将 `enable_actuator` 改为 `true`。继续保持吊装。`run_real_nav.sh` 使用不加载手柄的配置，适合本次单一外部速度源测试；它会尝试使能颈部舵机。颈部装配需要保持时，确认串口和使能成功，不能忽略脚本的头部松动告警。该脚本在已有 `cap_net_raw` 时不会覆盖第 4 节设置的 `cap_sys_nice`。

终端 A：

```bash
cd /path/to/F1/build
getcap ./aimrt_main
bash ./run_real_nav.sh
```

终端 B：

```bash
cd /path/to/F1
source scripts/ros2_source.sh
ros2 topic info -v /cmd_vel_limiter
ros2 topic pub -r 10 /cmd_vel_limiter geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

保持终端 B 的**唯一**零速度发布者持续运行。确认没有 Nav2、手柄、其他脚本等同时发布 `/cmd_vel_limiter`。速度超过 250 ms 未更新会自动归零，但不能把话题超时作为紧急停车手段。

终端 C 同样执行 `source scripts/ros2_source.sh`，待进程启动至少 1 秒后，**逐条执行**以下指令；每次等日志显示状态转换、机器人稳定，并至少间隔 1 秒：

```bash
ros2 topic pub --once /zero_mode std_msgs/msg/Float32 '{data: 0.0}'
ros2 topic pub --once /stand_mode std_msgs/msg/Float32 '{data: 0.0}'
ros2 topic pub --once /amp_mode std_msgs/msg/Float32 '{data: 0.0}'
```

应依次看到 `idle -> zero -> stand -> walk_amp`。进入 AMP 前若出现 `AMP entry rejected`，检查 IMU/关节反馈，不要反复强行切换。进入 AMP 后**零速度至少保持 1 秒**，观察策略从浅蹲站姿立即接管后的目标变化，确认无异常抖动、踝电机扭矩尖峰、关节触限或 `AMP fault: switched to keep mode`。

仅在零速和站姿检查通过后，继续在吊架内做极低速度测试。先停止终端 B 的零速度发布，再确认旧发布者已消失，由终端 B 启动下面的**唯一**测试发布者；初次示例为 `0.05 m/s`，逐级增速须由现场验证结果决定。

```bash
ros2 topic pub -r 10 /cmd_vel_limiter geometry_msgs/msg/Twist '{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

首次不要测试急转、后退边界指令或外力推扰。观察左右腿、足部离地和踝电机响应；出现拖步、震荡、限位碰撞、状态异常或急停需求时按第 7 节处理。

## 6. 一键脚本与手柄操作

第 5 节的手工指令是基准流程，理解每一步的作用后再使用本节快捷方式。两者不要混用：任何时刻 `/cmd_vel_limiter` 只允许一个发布者。

### 6.1 一键脚本 `scripts/run_amp_real.sh`

脚本串起：可选构建 → tmux 启动小脑（`run.sh`，即 `x1_cfg.yaml`，**含手柄模块**，并尝试使能颈部舵机）→ `/joint_states` 与 `/imu/data` 反馈检查 → 人工安全确认 → `zero → stand →（确认站稳）→ amp` → 键盘遥控。

```bash
cd /path/to/F1
./scripts/run_amp_real.sh --build   # 首次：先构建再启动
./scripts/run_amp_real.sh           # 直接启动（需已有 build/ 产物）
./scripts/run_amp_real.sh --attach  # 小脑已在运行，仅进入模式切换+遥控
```

要点：

- 需要 tmux。小脑运行在 tmux 会话 `f1_amp` 的 aimrt 窗口，日志用 `tmux attach -t f1_amp` 查看。
- 反馈检查不通过（话题无订阅者或无数据）时脚本终止；此时 `/amp_mode` 同样会被小脑的 AmpReady 入口检查拒绝。
- 脚本读取运行配置中的 `constant_cmd`。默认固定 1 m/s 时，进入前会明确提示，键盘只用于模式切换，速度键和空格归零无效；删除固定指令后，键盘以 50 Hz 持续发布 `/cmd_vel_limiter`，限幅与训练范围一致（vx∈[-0.5, 2.5]、vy±0.5、wz±1.5，小脑侧二次限幅兜底）。按键：`m/s/k/z/i` = amp/stand/keep/zero/idle 模式；方向键（或 `WASD`/`Q`/`E`）按 0.1 步进调速；空格归零；`Ctrl+C` 先发零速再触发 `/stand_mode` 安全退出。
- 模式切换采用 5 Hz 持续发布 2 s，规避 DDS 发现期丢包（与 `switch_x1_mode.sh` 同策略），与手工 `--once` 方式等效。
- `run.sh` 已改为执行 `setcap cap_net_raw,cap_sys_nice=ep`，同时提供 EtherCAT 和实时调度权限，失败时不会启动真机控制。
- 手柄若已连接，手柄模块同时处于运行状态，**不要同时使用键盘遥控和手柄**。

### 6.2 手柄操作

手柄模式经 `run.sh`（`x1_cfg.yaml`）启动；`x1_cfg_real_nav.yaml`（no-joy 配置）下所有按键和摇杆无效。按键映射定义在 `build/cfg/joy_stick_module/joy_x1.yaml`：

| 输入 | 功能 |
|---|---|
| B(1) → `/zero_mode` | zero / 复位 |
| A(0) → `/stand_mode` | stand；仍受普通模式 1 秒节流 |
| X(2) → `/walk_mode` | 12DOF 行走（上升沿 toggle 免死区速度直发模式） |
| Y(3) → `/walk_mode2` | 腿臂行走 |
| **Guide(8) → `/amp_mode`** | **进入 AMP（须从 stand 状态）** |
| Back(6) → `/keep_mode` | keep；**AMP 内退出立即生效** |
| RB(5) → `/plan_mode`、Start(7) → `/idle_mode` | plan / idle |
| LB(4) 按住 | 默认固定 AMP 中无效；外部速度模式下按住期间摇杆速度生效 |
| 左摇杆 前后 / 左右 | 默认固定 AMP 中无效；外部速度模式下为 vx / vy |
| 右摇杆 左右 | 默认固定 AMP 中无效；外部速度模式下为 wz |
| LT 左扳机 | RT 外部速度模式的 5 s 减速；不能用于固定 AMP 刹停 |
| RT 右扳机 | IMU 偏航闭环；固定 AMP 忽略其速度，测试期间不要使用 |

默认操作流程：启动至少 1 秒 → B(zero) 复位完成 → A(stand) 浅蹲站稳至少 3 秒 → 单次 Guide 进 AMP，立即持续固定 1 m/s。之后无需按键、LB 或摇杆；当前没有 1 秒软启动。外部速度模式需先删除 `constant_cmd`，再从零速、低速逐级验证。反馈不新鲜时按 Guide 无效（日志出现 `AMP entry rejected`），应先排查 IMU/关节反馈，不要反复强按。

安全行为与限制：

- 默认固定 AMP 不受 LB、摇杆、外部零速度或速度话题超时影响，停车必须切出 AMP。删除 `constant_cmd` 后，松开 LB 才会使外部指令 250 ms 超时归零。
- 手柄侧限幅只作用于外部速度模式；AMP 的固定 1 m/s 由控制器直接使用，并按训练范围限幅。
- **测试期间勿按 RT/LT**：RT 触发的 `/walk_mode` 被 AMP 状态机拒绝，固定 AMP 也忽略其闭环速度；LT 的速度斜坡不能停止固定指令，要用 A/Back 切出 AMP。
- 8 号键（Guide）是否上报取决于手柄驱动：首次联调先按键并用 `ros2 topic echo /amp_mode` 确认有输出；无输出则把 `joy_x1.yaml` 中 `buttons: [8]` 改为 `[9]` 或 `[10]`（摇杆下压），重新构建部署。

## 7. 退出、记录与放行判断

默认固定模式退出：按 A 切 stand，或 Back 切 keep；固定指令只在 AMP 激活时生效。不要使用零速度话题或松开 LB 停车。

以下零速度退出步骤仅用于第 5 节已删除 `constant_cmd` 的外部速度模式。正常退出：在终端 B 停止非零速度发布者，重新以 10 Hz 持续发布零速度，确认机械运动停止，然后从终端 C 发布 `/keep_mode`；`keep` 仍会保持当前位置，不是断电。若需完全退出控制，在机器人由吊架可靠支撑、`keep` 状态已确认且间隔至少 1 秒后，再切 `/idle_mode`，最后在终端 A 用 `Ctrl+C` 停止主进程并确认驱动器下使能。`idle` 发送零刚度/阻尼指令，**本身不调用执行器下使能**。AMP 内的 `/keep_mode` 和 `/idle_mode` 可绕过通常的 1 秒模式节流；进入 `keep` 后的普通模式切换仍受节流限制。**出现失控迹象时立即使用硬件急停，不等待 ROS 指令。**

```bash
# 终端 B：先 Ctrl+C 停止原发布者，再运行
ros2 topic pub -r 10 /cmd_vel_limiter geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
# 终端 C：确认机械运动停止后运行
ros2 topic pub --once /keep_mode std_msgs/msg/Float32 '{data: 0.0}'
# 若机器人已被吊架可靠支撑，且 keep 转换后已过至少 1 秒，再切 idle
ros2 topic pub --once /idle_mode std_msgs/msg/Float32 '{data: 0.0}'
```

现场保存并检查手柄启动的 `build/log/x1_rl_ctrl.log`（no-joy 启动为 `build/log/x1_rl_ctrl_real_nav.log`）、`build/test_logs/data_csv/walk_diag_*.csv`、`build/test_logs/data_csv/t_m/tm_obs_input_*.bin`、`build/test_logs/data_csv/ctrl_period_*.csv`。重点比较关节实测值与 `pos_des_raw_*` / `pos_des_lpf_*`、踝关节 `tau_des_*`、踝电机命令与反馈；无 LPF 时 `_lpf` 列代表经限位、限幅后的实际目标。`ctrl_period` 的 `miss_cnt` 是**唤醒迟到至少 100 µs**的计数，不等同于真正丢失 1 ms 控制周期，需结合 `late_us_max` 和现场时间序列判断。

放行到下一阶段前，至少确认：Linux 构建与模型哈希一致；29 关节和 IMU 坐标/零位一致；吊架内 `idle/zero/stand/AMP` 切换正确；1 kHz 输出与 50 Hz 策略配置相符；策略切入、关节限位、并联踝力矩与电机反馈符合预期；反馈断流/倾斜/推理错误时退出 AMP；硬件急停和断线保护可独立生效。当前 AMP 实现尚无机身高度或足底触地看门狗，对 DCU 重发旧传感器值也不能仅靠 ROS 接收时间识别；上述吊架检查通过后仍需补充现场风险评估，才能决定是否开展有保护的地面试验。
