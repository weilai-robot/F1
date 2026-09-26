# AMP部署操作说明

适用范围：`feat/amp-deploy` 分支、X1 29DOF、`x1_amp_soup59c_50hz.onnx`，部署目标为连接 X1 的 Linux 主控。以下命令均应在 **Linux 主控**执行。本说明只用于受保护的吊架/悬空联调；完成这些步骤不代表可以无保护地面行走。训练和模型对齐结果见 [AMP 部署审计](amp_deploy.zh_CN.md)。

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
| 同文件的 `rl_walk_amp` | `decimation: 20`、`action_scale: 0.25`、`settle_ticks: 50`、`policy_file` 为本次模型、`lpf_conf.position_enabled: false`、`lpf_conf.effort_enabled: false` |
| `cfg/x1_cfg_real_nav.yaml` | 加载 `ControlModule` 与 `DcuDriverModule`，不加载 `JoyStickModule`；现场没有其他节点写 `/cmd_vel_limiter` |

`initial_state: idle` 是 `rl_x1.yaml` 的**顶层**字段；建议显式设置，避免启动状态依赖 YAML 中状态的排列顺序。`ethercat.bind_cpu` 与控制器 `bind_cpu` 必须是该 Linux 主控上有效且合适的独立 CPU。首次启动前核对机身 IMU 的 body→world 四元数定义、29 关节反馈的方向/零位，以及并联踝双电机的传动符号和限幅。配置里的全零 `joint_offset` 不能代替实机标零。

50 Hz 策略每 20 个 1 kHz 控制周期推理一次。此模型训练和 sim2sim 验证未启用动作 LPF，所以 AMP 默认关闭位置目标及踝力矩 LPF，保留 1 秒切入姿态平滑、关节目标限位和踝关节力矩限幅。若需改变 LPF，须重新验证该配置的闭环行为，不要在首次真机联调时临时开启。

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

## 5. 吊装状态下使能并进入 AMP

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

应依次看到 `idle -> zero -> stand -> walk_amp`。进入 AMP 前若出现 `AMP entry rejected`，检查 IMU/关节反馈，不要反复强行切换。进入 AMP 后**零速度至少保持 1 秒**，确认从实测姿态平滑过渡至策略默认站姿，且无异常抖动、踝电机扭矩尖峰、关节触限或 `AMP fault: switched to keep mode`。

仅在零速和站姿检查通过后，继续在吊架内做极低速度测试。先停止终端 B 的零速度发布，再确认旧发布者已消失，由终端 B 启动下面的**唯一**测试发布者；初次示例为 `0.05 m/s`，逐级增速须由现场验证结果决定。

```bash
ros2 topic pub -r 10 /cmd_vel_limiter geometry_msgs/msg/Twist '{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

首次不要测试急转、后退边界指令或外力推扰。观察左右腿、足部离地和踝电机响应；出现拖步、震荡、限位碰撞、状态异常或急停需求时按第 6 节处理。

## 6. 退出、记录与放行判断

正常退出：在终端 B 停止非零速度发布者，重新以 10 Hz 持续发布零速度，确认机械运动停止，然后从终端 C 发布 `/keep_mode`；`keep` 仍会保持当前位置，不是断电。若需完全退出控制，在机器人由吊架可靠支撑、`keep` 状态已确认且间隔至少 1 秒后，再切 `/idle_mode`，最后在终端 A 用 `Ctrl+C` 停止主进程并确认驱动器下使能。`idle` 发送零刚度/阻尼指令，**本身不调用执行器下使能**。AMP 内的 `/keep_mode` 和 `/idle_mode` 可绕过通常的 1 秒模式节流；进入 `keep` 后的普通模式切换仍受节流限制。**出现失控迹象时立即使用硬件急停，不等待 ROS 指令。**

```bash
# 终端 B：先 Ctrl+C 停止原发布者，再运行
ros2 topic pub -r 10 /cmd_vel_limiter geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
# 终端 C：确认机械运动停止后运行
ros2 topic pub --once /keep_mode std_msgs/msg/Float32 '{data: 0.0}'
# 若机器人已被吊架可靠支撑，且 keep 转换后已过至少 1 秒，再切 idle
ros2 topic pub --once /idle_mode std_msgs/msg/Float32 '{data: 0.0}'
```

现场保存并检查 `build/log/x1_rl_ctrl_real_nav.log`、`build/test_logs/data_csv/walk_diag_*.csv`、`build/test_logs/data_csv/t_m/tm_obs_input_*.bin`、`build/test_logs/data_csv/ctrl_period_*.csv`。重点比较关节实测值与 `pos_des_raw_*` / `pos_des_lpf_*`、踝关节 `tau_des_*`、踝电机命令与反馈；无 LPF 时 `_lpf` 列代表经软启动、限位、限幅后的实际目标。`ctrl_period` 的 `miss_cnt` 是**唤醒迟到至少 100 µs**的计数，不等同于真正丢失 1 ms 控制周期，需结合 `late_us_max` 和现场时间序列判断。

放行到下一阶段前，至少确认：Linux 构建与模型哈希一致；29 关节和 IMU 坐标/零位一致；吊架内 `idle/zero/stand/AMP` 切换正确；1 kHz 输出与 50 Hz 策略配置相符；软启动、关节限位、并联踝力矩与电机反馈符合预期；反馈断流/倾斜/推理错误时退出 AMP；硬件急停和断线保护可独立生效。当前 AMP 实现尚无机身高度或足底触地看门狗，对 DCU 重发旧传感器值也不能仅靠 ROS 接收时间识别；上述吊架检查通过后仍需补充现场风险评估，才能决定是否开展有保护的地面试验。
