# 舞蹈输入与估计诊断日志

用于比较多次演示的起始状态、估计波动、接触判断和执行差异。需要更新主仓库及 `motion_control` 并重新编译。默认配置：

```yaml
controllers:
  dance_x1:
    diagnostics:
      enabled: true
      directory: test_logs/data_csv
      flush_ms: 100
```

从 `build` 目录执行 `bash ./run_dance.sh` 后，启动日志显示 `Dance diagnostics: ...`。文件位于：

```text
build/test_logs/data_csv/dance_diag_<Unix纳秒>.csv
build/test_logs/data_csv/dance_diag_<Unix纳秒>.csv.yaml
```

YAML 是初始化时实际传给舞蹈控制器的配置快照，含关节顺序、限位、估计器参数及模型/动作路径。它不包含二进制资产本体或主程序版本，采集时还应保留实际资产 JSON、主仓库及子模块提交号。`logger_start` 的 reason 为 `config_saved` 表示配置快照写入成功；失败时为 `config_snapshot_failed`。

## 采样和演示编号

- `event=policy`：每次 50 Hz 推理一条，来自该次推理使用的同一个 Snapshot。获取该帧关节命令后入队，重复读取命令不重复写行。
- `event=passive`：zero、stand、dance_ready 阶段以 50 Hz 记录；进入 keep 后再记录约 1 秒。idle 不持续采样。
- `event=state_change`：模式切换；`fault` 记录故障；`completed` 表示参考动作正常结束；`logger_start/logger_stop` 标记文件生命周期。
- `session`：每次进入 dance_ready 增加，包括取消的准备尝试；紧随其后的播放沿用该编号。未开始第一次准备时为 0。同一进程多次演示写在同一个文件，重启进程另建文件。
- `policy_frame`：仅有观测的推理/推理故障行填写，首帧为 0；其他行是 -1。`motion_time_s=policy_frame/50` 是参考动作时间，不是墙钟持续时间。
- 同一输入记录中的 `obs_raw_131..159` 是该帧推理前的上一帧 action；`action_<joint>` 是本帧 MLP 输出。

## 字段

| 字段 | 含义 |
| --- | --- |
| `wall_ns / steady_ns` | 日志采样时刻；分别是系统时钟和单调时钟，整数纳秒 |
| `joint/imu/body/anchor/velocity_stamp_ns` | 参与该次 Snapshot 的各路消息源时间；0 表示不可用 |
| `snapshot_valid` | 控制器是否接受了完整且有效的 Snapshot |
| `has_observation / has_action` | 是否有该次推理的输入/候选输出；故障行仍可能含 NaN/Inf |
| `command_valid` | 本行记录了 GetJointCmdData 生成的命令；不证明总线送达或电机实际执行 |
| `estimator_valid` | 本行拥有与身体里程计源时间相同的内置估计器诊断；外部状态模式不伪造此信息 |
| `imu_quat/gyro/accel_*` | 控制器接受的 IMU 姿态、角速度和含重力的比力 |
| `body_position_world/body_quat/base_velocity_body_*` | 身体位置/姿态，以及身体坐标系线速度 |
| `anchor_position_world/anchor_quat_*` | policy 使用的腰部上端锚点位姿 |
| `reference_position_world/reference_quat_*` | 已按本次起点和 yaw 对齐的参考锚点，仅推理行填写 |
| `anchor_error_world_*/anchor_error_norm` | 参考减实际估计锚点的世界系误差及三维范数；仍然记录，未重新引入退出阈值 |
| `est_acceleration_world_*` | 估计器惯性预测使用的世界系去重力加速度，尚未减去估计偏置 |
| `est_accel_bias_world_*` | 滤波器当前估计的世界系加速度偏置 |
| `est_position/velocity_variance_world_*` | 滤波器内部各轴方差，单位 m²、(m/s)²；不是实测误差方差 |
| `est_vertical_force_left/right` | 准静态腿部载荷估计，单位 N；不是足底传感器真值 |
| `est_contact_left/right` | 本体估计器是否采用该脚作为可信支撑，0/1 |
| `joint_position/velocity/effort_<joint>` | Snapshot 中的关节反馈，已转到 policy 模型坐标；effort 沿用 DCU 传动层转换 |
| `obs_raw_0..159` | 归一化前的完整观测，顺序见主部署说明 |
| `obs_normalized_0..159` | 实际输入 actor 的完整归一化观测 |
| `action_<joint>` | 本次 MLP 候选输出；故障行应结合 has_action 和 command_valid 使用 |
| `target_policy_<joint>` | 按关节限位裁剪后的 policy 坐标目标；无有效新命令的故障行可能是旧目标 |
| `command_position/effort/kp/kd_<joint>` | 该推理帧 GetJointCmdData 返回的命令，已乘 joint_sign，但尚未加顶层 joint_offset |
| `sequence/dropped_total` | 队列尝试序号及累计丢帧；队列满时丢日志，不等待磁盘 |

NaN 表示缺失/不适用，不能当成 0。外部 anchor/velocity 模式没有 base 位置和内置估计诊断时，这些字段保持 NaN。校准未完成或估计失效时仍尽量记录最近接受的关节/IMU，其源时间与 snapshot_valid 必须一起看。

普通关节使用电机内部 PD，`command_effort=0` 不是实际电机扭矩为 0；看 `joint_effort` 和位置目标。踝关节是软件 PD，command_effort 是该推理帧生成的限幅后力矩。日志不逐一记录其余 1 kHz 踝力矩更新，也不记录全部 1 kHz 原始估计器输入，因此不能用这份 50 Hz CSV 精确重放整个估计器。

## 对比多次演示

按文件和 session 分组，将 policy_frame=0 对齐，重点看播放前的 dance_ready 及播放前 3 秒：

1. 站稳窗口的线速度均值、标准差和位置趋势；均值偏差与慢漂移需要和抖动分开统计。
2. accel_bias 是否随尝试发生变化，接触是否频繁切换，载荷是否在阈值附近跳变。
3. joint/imu/body 等源时间差和年龄，检查估计状态与其余输入是否错时。
4. 估计速度/锚点、归一化输入和 action 的变化是否先于身体倾斜；随后再检查关节跟踪和力矩。

跨次比较世界系位置时要统一首帧位置和朝向。动态动作中的信号标准差包含真实运动，不能直接称为估计误差方差；确定真实估计误差仍需要外部参照。

## 落盘与收集

控制线程只复制固定尺寸记录到预分配队列；独立线程格式化 CSV，并默认每 100 ms flush。正常退出会排空队列，并写 logger_stop；不用等待旧控制周期统计的 10 秒窗口。

整机断电、SIGKILL 或磁盘错误可能留下缺失的结束标记和尾部数据。flush 不等于断电持久化保证。手动下电后出现 stale 也可能是人为停止的结果，需结合现场记录，不能自动判为失稳原因。

演示后收集本次 `dance_diag_*.csv`、对应 `.csv.yaml`、`ctrl_period_*.csv` 和 `build/log/x1_rl_ctrl_dance.log*`，并注明人工停止时刻及原因。可快速查看每次尝试记录量：

```bash
python3 - <<'PY'
import csv, glob, collections
for path in sorted(glob.glob('build/test_logs/data_csv/dance_diag_*.csv')):
    counts = collections.Counter()
    with open(path) as f:
        for row in csv.DictReader(f):
            if row['event'] == 'policy':
                counts[row['session']] += 1
    print(path, dict(counts))
PY
```

## 验证范围

独立写盘核心在 Mac 使用 C++17、ASan/UBSan 和 ThreadSanitizer 检查：字段列数/精度、CSV 转义、短时 flush、多次演示排空、队列满计数及路径错误；估计诊断输出的加速度/偏置/协方差也通过 ASan/UBSan 检查。完整控制器回归新增实际 actor 输入/命令一致性、重复读命令不重复记录、演示编号、故障详情、配置快照及缺失值检查，仍需在 Ubuntu 的 ROS/Eigen/gtest 环境编译运行。真机上的主循环额外开销、磁盘吞吐和日志完整性也需要现场确认。
