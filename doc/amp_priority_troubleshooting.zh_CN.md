# 左臂旧反馈与腿部超调：优先排查及采集流程

日期：2026-10-14。基线 motion_control `47b7125`。本次不修改 AMP 策略、速度、IMU 映射、方向、kp/kd 或限位；新增独立诊断配置、底层 RX/TX 快照及分析工具，修复已发现的缓存/校验问题。真机根因需要下一次原始诊断数据确认。

**目前已确认的现象**

左臂：两份行走日志中逻辑左臂 7 关节的 position/velocity/effort 在 0.26 / 0.82 s 起停止更新，其他关节及 IMU 仍变化。左肩仍报非零速度，但位置不变。若现场配置与仓库一致，该组属于 `body/ECAT 1/channel_1/CAN ID 1–7`。优先核对这一支路的 CAN 接线/供电、DCU 转发和 SDK 接收数据；此时不能只用 ROS 话题频率证明反馈新鲜。

左膝：目标摆幅约 43°，实际约 55°；最大实际角度比停止前记录到的任何目标高约 10°。目标回落时膝仍在继续屈曲，存在动态超调表现。停止后的尾段已排除。基线统计：[JSON](analysis/amp_20261013/priority_baseline.json)、[文本](analysis/amp_20261013/priority_baseline.txt)。

**代码审查结果与修复**

1. EtherCAT `WorkLoop` 原来在接收失败/WKC 不足时也把输入缓冲复制到 SDK。现在只有完整 WKC 接收才刷新节点反馈，避免把旧 PDO 重复当作有效接收。某一 CAN 支路冻结但 EtherCAT 正常时，仍需要逐槽诊断；该修复不能被宣布为已经修复左臂硬件故障。
2. 原来 position、velocity、effort 分别取得接收锁，有机会来自不同 PDO。现在驱动使用一次接收锁读取三者及有效 PDO 接收时间的快照，诊断也保留同一锁内的原始 RX 字节。有效 PDO 不再到达时，不再重复发布带新时间戳的 JointState，现有 AMP 反馈超时检查因此可以看到断流。静止但持续收到有效 PDO 的关节照常发布；CAN 单支路旧测量而 PDO 仍更新的情况不会凭静止自动触发此规则。
3. SDK 现在拒绝非法通道、CAN ID 不在 1–8、重复路由/名字，驱动检查初始化返回值；缺少数组项或非有限数的 JointCommand 被拒绝。避免配置/报文错误被静默带入链路。
4. 新增最后一次复制到 SOEM 输出缓冲的原始 TX 槽，独立于当前编码前缓存。保留该次目标、kp/kd、前馈值、发送掩码和命令修订计数；行政指令不误解码成 MIT。L28 单独标记位置/电流限制协议，其 kp/kd 不被伪报为实际下发。

SDK 接收包只有每通道 64 字节、IMU 及保留字节，没有已定义的逐电机 CAN 收包时间/序号。R52/R86 字段包含一个 heartbeat bit，但目前没有确认它的切换周期/语义。因此本次分开记录 **PDO 到达年龄、完整 8 字节内容变化年龄、位置/速度/力矩前 5 字节变化年龄**。它们都不能单独替代真正 CAN 收包年龄。静止关节长期重复数据是允许的；分析仅将“测量字节长期不变而速度持续非零”标为可疑，未新增凭此自动退出 AMP 的规则。

诊断采集期间若出现这种运动反馈矛盾，会按 ECAT/通道每 2 s 打印一次 `DCU suspicious repeated moving feedback` 警告。若 AMP 测试中持续出现，应按已有停止流程结束采集，先定位反馈；警告不会自动改电机命令。

**下肢的具体优先项**

| 项目 | 已验证 main 的名义 kp/kd | AMP 名义 kp/kd |
|---|---:|---:|
| hip pitch | 30/3 | 120/4 |
| knee pitch | 100/8 | 150/5 |

同一惯量 I、理想 PD 的简化模型中，阻尼比 `ζ = kd/(2√(kp·I))`。因此 AMP/main 的膝相对阻尼约 `(5/8)/√(150/100) = 0.51`，髋 pitch 约 `(4/3)/√(120/30) = 0.67`。这是明确的增益变化方向，值得优先核对制动响应；它不等于已测出真实阻尼比。29DOF 耦合、接触、1 m/s 动作、50 Hz 目标、LPF 关闭和固件限制同时改变了闭环条件，不能直接把该模型当作新的调参结果。

新的诊断将验证：

- 编码后的膝 kp/kd 是否约为 150/5，以及与上层缓存的差异；正常量化约 kp 每码 0.122、kd 每码 0.00195。
- 目标开始回落、膝仍屈曲时，原生速度、反馈 effort、状态位及发送槽是否继续更新。
- 反馈 effort 是否经常靠近主机解码范围端点。旧两次数据中，左膝 ≥98 的帧比例分别 0 / 0.84%，右膝为 0；不足以认定持续饱和，也不排除瞬态或内部限流。
- 主机 `host_pd_effort = ff + kp·(target−q) + kd·(target_vel−dq)` 与反馈 effort 的时间过程。该值是按主机量程推算的理想 PD，**不是实测电机输出**，差异不能直接用来拟合/宣布真实 kp/kd。
- 固件与主机的 MIT 量程是否一致，特别是 kd 最大值。主机默认 kd_max=8；如果固件实际按 5 解码，名义 kd=5 对应约 3.125。该例只是量程不一致的假设，需按固件配置确认。

当前 `GetMode()` 没有可信的 R52/R86 固件模式读回，不能用默认 MODE_CURRENT 作结论。TX MIT 格式表示主机发送了该格式，不证明电机已进入 MIT 模式。也不能将反馈解码 ±100 范围直接当作电机内部 PD 的总力矩上限。

**服务器更新与离线检查**

```bash
cd ~/Robot_github/F1
git pull --ff-only origin feat/amp-deploy
git submodule update --init --recursive
bash scripts/test_amp_priority_diag.sh
bash scripts/test_mit_encoding.sh
bash scripts/build.sh
```

需要重新编译新 SDK/驱动；仅拉取 YAML 或沿用旧二进制不会有新诊断。构建安装会更新 `build/cfg`，编译前保存现场配置，编译后核对网卡、拓扑及已标定的 joint_offset；避免直接用旧配置覆盖此次软件/方向修正。诊断生成器保留其读到的现场配置字段，原文件不改写。脚本 start 会检查 `libpkg1.so` 中的诊断标识。Python 配置生成通常需要系统 python3-yaml；分析需要 NumPy/pandas，SciPy 用于周期指标。原有上肢客户端仍需 ROS2/rclpy 和生成消息环境。

**第一阶段：先静态定位左臂**

停止旧控制进程，机器人可靠支撑。终端 A：

```bash
bash scripts/run_amp_priority_diag.sh start --scenario static
```

它复用已有上肢静态 PD 测试，使用默认 0.10 rad 增量，生成独立配置并启用 DCU 诊断。不会加载 AMP；不会自动执行全身行走。原现场网卡、路由、传动方向、标零配置保留。

终端 B，先测左肩，再测左肘；不同时操作多个测试客户端：

```bash
bash scripts/run_upper_direction_test.sh test --joint left_shoulder_pitch_joint
# 上一项完成后：
bash scripts/run_upper_direction_test.sh test --joint left_elbow_pitch_joint
```

终端 C 可只读检查整个上肢：

```bash
bash scripts/run_upper_direction_test.sh diagnose
```

测试后同时保留 `build/test_logs/data_csv/upper_direction_*.csv/.json`、`upper_link_diagnose_*.json`（具体路径以工具打印为准）及 `build/test_logs/dcu_diag/dcu_diag_*.csv`。DCU 默认 50 Hz、120 s，独立 SCHED_OTHER 线程写文件；1 kHz 控制/反馈循环不做 CSV 格式化或磁盘写入。到期只停止记录，**不停止机器人控制**；超过 120 s 需用 `--duration 300` 重启采集进程。

| 新日志观察 | 下一层定位 |
|---|---|
| body PDO 序号继续增长、rx_age 很小；左臂测量序号不再增长，速度却持续非零 | 数据已经在 DCU→主机的原始槽中重复，优先查 CAN1、DCU CAN 接收/转发及电机反馈；单靠 ROS 转发无法修复 |
| 原始 8 字节变化，仅 heartbeat/status 变化，测量前 5 字节停更 | 区分状态/心跳更新与真正运动反馈更新，需 CAN 原始抓包/固件核对 |
| 原始测量槽及 SDK q/dq/effort 在变化，上层 joint_state 却不变 | 主机传动、发布、订阅、映射或现场旧二进制/配置 |
| tx_pdo_seq 增长，原始 TX 目标不变，而上层缓存目标变化 | SDK 调用、发送快照/控制掩码、实际配置或采样时序；结合 tx_command_seq 核对 |
| 原始 TX 目标变化，反馈正常更新但机械未响应 | 固件模式/使能、实际绑定、驱动限制或机械原因 |
| rx_age 增长/tx_pdo_seq 不再增长，多个支路同时异常 | EtherCAT 接收、主机 IO 调度或从站恢复 |

status/error/heartbeat 字段是原始接收缓存中的位，不能作为额外实时 ACK。目标缓存快照和 SDK 快照时间接近但不是同拍锁定，目标快速变化时不要仅凭单行缓存误差判断编码错误。原始 TX 快照是**放入 SOEM 输出缓冲的字节**，不等于已确认在 CAN 线上被电机收到。

静态阶段应确认所选关节与原生电机目标一致、正确部位有位移、位置和速度一致，左臂不再出现成组旧测量。完成后再进行 AMP 对照。

**第二阶段：记录腿部超调，不同时改参数**

结束静态测试，停止其 aimrt_main。终端 A：

```bash
bash scripts/run_amp_priority_diag.sh start --scenario amp
```

该配置保留现有 AMP 行为：B→zero，A→stand，站稳等待至少 3 s，单次 Guide→AMP，持续原 `[1,0,0]` 指令。必须在第一阶段左臂反馈已正常后进行；这轮用于取得原增益的基线，不同时改 kp/kd、频率或限位。按当前支撑/停止流程结束，不把人为停止之后的数据当作步态故障。

保留同一次进程的以下文件：

- `build/test_logs/dcu_diag/dcu_diag_*.csv`：原始槽、序号/变化年龄、编码后命令、反馈。
- `build/test_logs/data_csv/walk_diag_*.csv`：策略目标和全身观测。
- `build/test_logs/ecat_stats/ecat_stats_*.csv`：同期 IO 统计；既有 DC 首样本/minimum 统计缺陷仍需按已有报告处理。
- `build/cfg/amp_priority_manifest.json`：实际来源配置哈希、路由与零偏。

**分析命令**

静态只看 DCU 链路：

```bash
bash scripts/run_amp_priority_diag.sh analyze \
  --dcu-log ./build/test_logs/dcu_diag/dcu_diag_实际文件.csv \
  --out /tmp/amp_priority_static
```

AMP 按同一进程的策略日志限定有效窗口：

```bash
bash scripts/run_amp_priority_diag.sh analyze \
  --dcu-log ./build/test_logs/dcu_diag/dcu_diag_实际文件.csv \
  --walk-log ./build/test_logs/data_csv/walk_diag_实际文件.csv \
  --control-config ./build/cfg/control_module/rl_x1.yaml \
  --out /tmp/amp_priority_walk
```

生成 `priority_summary.json` 与 `priority_summary.txt`，自动汇总可疑测量重复支路、PDO 接收年龄、发送槽、编码后增益与髋膝摆幅。静态日志没有 walk 窗口时会分析整个记录，需自行区分人工停机尾段。`--walk-log` 自动使用停止前全身反馈最后变化窗口，仍需结合实际操作确认其含义。

**验证范围**

无硬件测试直接使用生产 SDK，验证重复 PDO 不刷新内容变化时间、心跳不刷新测量变化时间、不同通道隔离、非法/重复路由拒绝、最后实际拉取 TX 与待发缓存分开、R86 150/5 解码、L28 协议标记及并发反馈快照一致。Python 检查配置生成保留现场硬件/AMP参数、dry-run 不写文件、静止重复不误报，以及生产诊断 worker 的有界 CSV 输出。既有上肢 17 路 SDK 路由及 MIT 编码回归也已通过。

本机未连接机器人，完整 Linux AimRT/ROS2 构建及硬件调度/执行效果需要服务器完成。没有宣称已经恢复左臂 CAN 反馈或消除腿部超调；新数据用于把这两项问题定位到具体链路，再决定可验证的修复或单变量调参。
