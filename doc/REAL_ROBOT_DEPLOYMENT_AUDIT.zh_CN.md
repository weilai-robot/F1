# F1 实机部署审查与上机基线

> 审查日期：2026-08-05  
> 目标主控：Intel 第 12 代 Core i7（x86_64）  
> 目标系统：Ubuntu 22.04 LTS + ROS 2 Humble + PREEMPT_RT  
> 当前结论：运动控制可按“电机未使能 → 总线验证 → 低风险使能”分阶段部署；完整真机自主导航在修复本文列出的阻塞项之前不应直接上机。

## 1. 文档目的

本文记录当前 F1 仓库针对 Intel 第 12 代 Core i7 主控的实机部署审查结果，作为后续代码修复、主控环境准备、实时性调优和上机验收的统一基线。

本文重点覆盖：

- 当前代码和子模块版本；
- 运动控制与真机导航的可部署状态；
- 已确认的部署阻塞项和安全风险；
- 1 kHz 控制线程与 EtherCAT IO 线程的实时调度、优先级和绑核方案；
- 首次上机的分阶段操作与验收命令。

## 2. 当前版本基线

审查时仓库状态如下：

| 组件 | Commit | 说明 |
|---|---|---|
| F1 集成仓库 | `bb50886de7f65336cdd9da3abf035a43a057d2db` | `main` 相对 `origin/main` ahead 1 |
| motion_control | `30d582317b59ac6bfac46418e7464289bddaaac4` | detached HEAD，由集成仓库锁定 |
| navigation | `ec6fd256eeb4cc03d6f4e350af0e3c9080d0ce95` | `nav/dev-gt-odom` |

部署时必须锁定并核对这三个版本，避免主控运行的代码与本文分析对象不一致：

```bash
git checkout bb50886de7f65336cdd9da3abf035a43a057d2db
git submodule update --init --recursive
git submodule status --recursive
```

仓库内附带的 ONNX Runtime、Ruckig 和 MuJoCo 动态库均为 Linux x86-64 ELF，架构上适配 Intel 第 12 代 i7。`libruckig.so` 最高要求到 `GLIBC_2.33`，Ubuntu 22.04 的 glibc 版本满足该要求。

## 3. 平台与部署方式

推荐基线：

- Ubuntu 22.04 LTS amd64；
- ROS 2 Humble；
- PREEMPT_RT 实时内核；
- 裸机运行 motion_control 和真机导航，不使用当前仿真 Docker 镜像驱动机器人硬件；
- EtherCAT 使用独立有线网卡，Livox 使用另一个网口或独立网络。

ROS 2 Humble 将 Ubuntu 22.04 amd64 列为 Tier 1 平台：

- <https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html>

Ubuntu 实时内核和实时调优参考：

- <https://documentation.ubuntu.com/real-time/latest/>

第 12 代 Intel CPU 使用性能核（P-core）与能效核（E-core）的混合架构。平均 CPU 利用率足够不代表 1 ms 周期可满足要求，最终准入依据必须是调度延迟、控制周期抖动和 EtherCAT WKC/DC 状态。

## 4. 部署就绪度结论

### 4.1 运动控制

运动控制具备基本真机入口，但首次部署前必须完成以下事项：

1. 将 `enable_actuator` 暂时设为 `false`；
2. 修改 EtherCAT 网卡名和 CPU 绑定；
3. 授予 `CAP_NET_RAW`，并通过 systemd `LimitRTPRIO` 或临时 `CAP_SYS_NICE` 提供实时优先级额度；
4. 为 `rl_control_pub_thread` 配置 `SCHED_FIFO` 与独立 P 核；
5. 验证 DCU/驱动器的硬件命令超时看门狗；
6. 在电机未使能状态确认总线、从站、IMU、关节状态和实时调度均正常；
7. 吊装并确认急停有效后，才允许进入电机使能阶段。

### 4.2 完整真机自主导航

当前不具备直接上机条件。主要阻塞包括：

- 真机启动脚本实际启动仿真导航入口；
- 真机 FastLIO `/Odometry` 与 `odom_bridge.py` 的订阅消息类型不匹配；
- 真机 MPPI 角速度上限过大；
- 真机地图、雷达 IP、外参和 TF 高度尚需按实际机器人确认。

## 5. 已确认问题

### P0-1：真机导航启动了仿真入口

`scripts/run_nav_real.sh` 当前执行：

```bash
ros2 launch humanoid_sim navigation.launch.py use_sim_time:=False
```

但 `navigation/planning/humanoid_sim/launch/navigation.launch.py` 内部：

- 使用 MuJoCo 地图与 `nav2_mujoco.yaml`；
- 包含 `tf_bridge.launch.py`；
- 将 Nav2 `use_sim_time` 硬编码为 `True`。

命令行传入 `use_sim_time:=False` 不能覆盖各节点内部的硬编码值。真机应使用并修复：

```bash
ros2 launch humanoid_sim navigation_real.launch.py
```

### P0-2：真机里程计消息类型不匹配

`tf_bridge_real.launch.py` 将 `input_topic` 配置为 `/Odometry`，预期接收 FastLIO 的 `nav_msgs/msg/Odometry`。

但当前 `odom_bridge.py` 是 MuJoCo Ground Truth 专用实现，固定按以下类型创建订阅：

```python
Float64MultiArray
```

结果是 FastLIO `/Odometry` 无法与该订阅建立连接，真机不会得到有效的 `/odom` 和 `odom -> base_footprint` TF。

修复方向：

- 恢复或新增独立的 `nav_msgs/Odometry` 真机回调；
- 仿真 GT 与真机 FastLIO 使用两个节点或显式 `input_mode`，不能只靠修改话题名切换；
- 保留 `/cmd_vel -> /cmd_vel_limiter` 的限速与加速度限制；
- 用 `ros2 topic info -v /Odometry` 验证类型和 QoS。

### P0-3：实时调度权限不足

EtherCAT 代码使用：

```cpp
pthread_setschedparam(pid, SCHED_FIFO, ...)
```

这里需要区分两类互不等价的权限：

- `CAP_NET_RAW`：允许进程打开 EtherCAT 使用的原始网络套接字；没有它，普通用户通常无法建立 EtherCAT 通信；
- 实时优先级额度：允许线程从普通的 `SCHED_OTHER` 提升为 `SCHED_FIFO`。它可以来自 root、`CAP_SYS_NICE`、systemd 的 `LimitRTPRIO`，或登录会话的 `RLIMIT_RTPRIO`。

现有 `motion_control/install/linux/bin/run.sh` 仅设置：

```bash
sudo setcap cap_net_raw=ep ./aimrt_main
```

这足以让普通用户打开 EtherCAT 原始套接字，但不一定允许 EtherCAT IO 线程提升为 `SCHED_FIFO:90`。因此，“之前单独运行 `motion_control` 时 EtherCAT、DCU 和电机都能正常动作”与“实时调度没有生效”并不矛盾：普通的 `SCHED_OTHER` 线程在系统空闲、负载较小的 Intel i7 上也可能长期表现正常。该结果证明功能链路基本可用，但不能证明系统在 FastLIO、Nav2、日志、磁盘 IO 和网络 IRQ 同时工作时仍满足 1 ms 最坏时延。

此外，当前实现存在一个会掩盖权限错误的返回值检查缺陷：

```cpp
if (pthread_setschedparam(pid, SCHED_FIFO, &s_parm) < 0) {
  // ...
}
```

`pthread_setschedparam()` 按 pthread 约定在成功时返回 `0`，失败时直接返回非零错误码，例如 `EPERM`，通常不是 `-1`。因此当前 `< 0` 判断可能把失败当作成功并打印设置成功日志；同一文件中的 affinity 错误日志也应使用函数返回的错误码，而不是 `errno`。

正确写法应为：

```cpp
const int rc = pthread_setschedparam(pid, SCHED_FIFO, &s_parm);
if (rc != 0) {
  LOG_ERROR("Thread %s set SCHED_FIFO:%d failed: %s",
            name.c_str(), rt_priority, strerror(rc));
  return false;
}
```

`motion_control/module/dcu_driver_module/xyber_controller/xyber_api/src/ethercat_manager.cpp` 的 `WorkLoop()` 当前还忽略了 `SetRealTimeThread()` 的返回值，`Start()` 启动线程、等待 100 ms 后便返回成功，因而实时调度失败不会传播到模块初始化或执行器使能阶段。生产模式应在执行器使能前确认 EtherCAT 实时线程已成功完成调度和绑核；开发模式如果允许降级为 `SCHED_OTHER`，也必须输出明确告警。

临时手工验证可以给二进制增加 `CAP_SYS_NICE`：

```bash
sudo setcap cap_net_raw,cap_sys_nice=ep ./aimrt_main
getcap ./aimrt_main
```

但 `CAP_SYS_NICE` 的范围较宽，不只影响本进程的一个线程。生产部署优先使用 systemd 的 `LimitRTPRIO` 精确授予实时优先级额度，详见 6.5 节。

注意：再次执行只包含 `cap_net_raw` 的旧 `run.sh` 会覆盖并移除已有的 `CAP_SYS_NICE`。反过来，即使已经授予权限，线程也不会自动成为实时线程；代码仍必须显式调用 `pthread_setschedparam()`，Control executor 还必须单独配置调度策略。

现场结论必须以 `chrt -p <TID>` 或 `ps` 显示的实际线程策略为准，不能仅依据程序日志或“动作看起来正常”。

### P0-4：启动时默认使能全部执行器

`motion_control/module/dcu_driver_module/cfg/dcu_x1.yaml` 当前配置：

```yaml
enable_actuator: true
actuator_debug: true
```

`DcuDriverModule::Initialize()` 在初始化 EtherCAT 后立即遍历并使能全部执行器。当前代码中未发现对 `/joint_cmd` 新鲜度的明确软件看门狗；正常 Shutdown 会执行 `DisableAllActuator()`，但异常退出、进程卡死或命令源停止时的安全行为仍依赖 DCU/驱动器本身。

首次上机必须使用：

```yaml
enable_actuator: false
actuator_debug: false
```

其中关闭 `actuator_debug` 可避免在 1 kHz 下额外发布完整执行器状态和命令，降低非必要负载。

在重新设置 `enable_actuator: true` 前，必须确认：

- 硬件急停确实切断或禁止执行器输出；
- DCU/驱动器具有命令超时自动失能或安全保持机制；
- 进程 `SIGTERM`、`SIGKILL`、网线断开、控制线程停止时的实际行为已经测试。

### P1-1：1 kHz Control MainLoop 未配置实时策略

控制循环运行在 `rl_control_pub_thread` 的 `simple_thread` executor 上，但现有 `x1_cfg.yaml` 没有为其配置调度策略或绑核。

当前循环通过 `sleep_until` 执行 1 ms 绝对周期，这一结构适合做周期控制，但在线程仍为 `SCHED_OTHER` 时会受 ROS2、日志、导航、IRQ 和系统后台任务抢占。

具体配置见第 6 节。

### P1-2：控制循环存在超周期追赶风险

当前 `MainLoop()` 每轮执行：

```cpp
next_iteration_time += period;
std::this_thread::sleep_until(next_iteration_time);
```

如果一次 ONNX 推理或锁竞争超过一个周期，下一次 `sleep_until` 可能立即返回，形成连续追赶执行。在线程改成高优先级 `SCHED_FIFO` 后，这种情况可能长时间占满实时核。

建议：

- 显式使用 `std::chrono::steady_clock`；
- 若完成时间已经跨过下一个周期，则跳过丢失周期并重新对齐；
- 统计 wake-up lateness、单周期执行时间和 missed deadline 数量；
- 禁止在每周期内输出日志。

### P1-3：实时循环仍包含动态分配和锁竞争

当前代码中：

- `GetJointCmdData()` 每周期创建并 resize ROS 消息；
- ONNX 推理路径创建临时 `std::vector<Ort::Value>`；
- `RLController::Update()` 每 `decimation=10` 个控制周期在 Control executor 上同步调用一次 `session_ptr_->Run()`，即 1 kHz 控制循环中的约 100 Hz 策略推理；
- ONNX Runtime 当前只调用 `SetInterOpNumThreads(1)`，没有显式设置 `SetIntraOpNumThreads(1)`，算子内部并行仍可能使用 ORT 工作线程；
- 控制器输入通过多个 `shared_mutex` 与 ROS 回调共享；
- 状态切换和控制器重启可能与控制循环并发。

配置 `SCHED_FIFO` 只能改善调度优先级，不能自动将循环变成硬实时。后续优化应包括：

- 预分配并复用 JointCommand 和 ONNX 输入输出；
- 使用无阻塞快照、双缓冲或有界数据交换替代控制线程中的不可控锁等待；
- 使用 `mlockall(MCL_CURRENT | MCL_FUTURE)` 并预触页；
- 将日志、文件 IO 和诊断数据落盘放在非实时线程；
- 对实际最坏执行时间而不是平均执行时间做验收。

### P1-4：真机导航速度参数高于近期仿真基线

`nav2_real.yaml` 当前主要限制：

```yaml
vx_max: 0.5
vx_min: -0.2
wz_max: 1.2
az_max: 2.0
```

近期 MuJoCo 导航配置约为：

```yaml
vx_max: 0.4
vx_min: -0.1
wz_max: 0.2
az_max: 1.0
```

真机首次测试应使用更保守上限，并确保最终 `/cmd_vel_limiter` 还有独立绝对值和加速度限幅。未经吊装验证不得直接使用 `wz_max: 1.2`。

### P1-5：构建和打包脚本不一致

已确认：

- `scripts/build_all.sh` 使用 `cd "$SCRIPT_DIR/navigation"`，实际会寻找不存在的 `scripts/navigation`；
- `scripts/build_nav.sh --no-livox` 虽解析该参数，但又通过 `"$@"` 将其传给 colcon，可能报未知参数；
- `XYBER_X1_INFER_SIMULATION` 虽在顶层声明，但 `motion_control/CMakeLists.txt` 与 `pkg1` 仍无条件编译、链接 `sim_module`；
- 当前 `--no-sim` 实际不能移除 MuJoCo/GLFW/DART 编译依赖；
- README 中 GCC 13、ONNX Runtime 和若干依赖说明与仓库实际构建方式不完全一致。

修复前应分别构建：

```bash
./scripts/build.sh clean
./scripts/build_nav.sh
```

不要使用当前 `scripts/build_all.sh` 作为真机部署入口。

## 6. 1 kHz 实时调度与绑核方案

### 6.1 推荐优先级与 CPU 分工

| 工作负载 | 调度策略 | 建议优先级 | CPU 分配 |
|---|---|---:|---|
| EtherCAT IO | `SCHED_FIFO` | 90 | 独占一个 P-core |
| Control MainLoop | `SCHED_FIFO` | 80 | 独占另一个 P-core |
| RL/ONNX 推理（当前同步方案） | 继承 Control 的 `SCHED_FIFO` | 同为 80 | 在 Control P-core 上执行；ORT 限制为单线程 |
| RL/ONNX 推理（可选异步方案） | `SCHED_FIFO` | 70 | 仅在同步推理无法满足 1 ms WCET 时，独占第三个 P-core |
| NIC IRQ | PREEMPT_RT threaded IRQ | 按实测调整 | 不得干扰 Control 核 |
| ROS2 回调、导航、日志 | `SCHED_OTHER` | 默认 | 其余 P/E 核 |

EtherCAT IO 优先级高于控制计算，优先保证每个 1 ms 总线周期能够收发。Control 与 EtherCAT 不应绑定到同一逻辑 CPU，也不应绑定到同一物理核的两个超线程。

#### ONNX 推理为何不是默认分配到第三个核

当前代码不是“1 kHz Control 线程 + 独立 ONNX 线程”架构，而是：

```text
Control MainLoop（1 kHz）
  -> controller->Update()
     -> 每 10 个周期 ComputeObservation()
     -> 同步 session_ptr_->Run()
     -> 动作后处理并发布 JointCommand
```

因此 ONNX 推理当前属于 Control MainLoop 的最坏执行时间（WCET），不是可以独立设置 affinity 的工作负载。`decimation=10` 只表示策略每约 10 ms 更新一次；由于推理同步阻塞 1 kHz 线程，发生推理的那个控制周期仍必须在 1 ms deadline 内完成。不能因为策略频率是 100 Hz，就允许同步 `Run()` 占用接近 10 ms。

当前 `LoadModel()` 没有注册 CUDA、TensorRT 或 OpenVINO Execution Provider，实机上使用的是默认 CPU 推理路径；线程配置仅包含：

```cpp
sessionOptions.SetInterOpNumThreads(1);
```

这没有显式限制 intra-op 算子并行。ORT 可能让 Control 实时线程等待在其他 CPU 上运行的普通工作线程，形成不可控抖动或优先级反转。对于当前小型策略网络，首选确定性的单线程配置：

```cpp
Ort::SessionOptions sessionOptions;
sessionOptions.SetExecutionMode(ORT_SEQUENTIAL);
sessionOptions.SetIntraOpNumThreads(1);
sessionOptions.SetInterOpNumThreads(1);
sessionOptions.AddConfigEntry("session.intra_op.allow_spinning", "0");
sessionOptions.AddConfigEntry("session.inter_op.allow_spinning", "0");
```

这样 `session_ptr_->Run()` 主要由调用它的 Control 线程在绑定的 P-core 上完成，不再依赖一个跨核 ORT 计算池。是否关闭 spinning、单线程是否更快，仍需用目标模型和目标 i7 实测；实时部署优先比较 P99.9 和最大时延，而不是只比较平均推理吞吐量。

建议分别记录：

- 单次 `session_ptr_->Run()` 的均值、P99、P99.9 和最大耗时；
- `ComputeObservation + Run + 动作后处理` 的完整策略路径耗时；
- 推理周期与非推理周期的 Control MainLoop 总耗时；
- deadline miss 是否集中在每第 10 个周期；
- ORT 单线程和默认 intra-op 配置下的 CPU 迁移及尾延迟差异。

初始工程目标可设为：完整 Control 周期 P99.9 小于 `800 us`、最大值小于 `1 ms`，为唤醒抖动和发布路径留出余量。该数值是首轮准入线，最终应按整机压力测试结果收敛。

只有当单线程同步推理在优化、预分配和锁消除后仍不能满足 1 ms WCET，才考虑异步拆分：

- 新建约 100 Hz 的 policy inference 线程，绑定第三个独立 P-core，建议 `SCHED_FIFO:70`；
- 1 kHz Control 线程通过无锁双缓冲读取最近一次完整 action，不等待推理线程；
- 为 action 设置序号、时间戳和最大允许陈旧时间，超时进入已验证的安全策略；
- 验证异步带来的一周期策略延迟不会破坏训练时假设和闭环稳定性；
- 不把异步 ONNX 线程绑定到 EtherCAT/Control 的 SMT sibling，也不让它使用全核 ORT 线程池。

异步化不是单纯的绑核调整，而是控制数据流和时序语义变更，不能在首次实机部署前未经回归验证直接启用。

### 6.2 配置 Control executor

在 `motion_control/install/linux/bin/cfg/x1_cfg.yaml` 中修改：

```yaml
aimrt:
  executor:
    executors:
      - name: rl_control_pub_thread
        type: simple_thread
        options:
          thread_sched_policy: SCHED_FIFO:80
          thread_bind_cpu: [6]  # 示例，必须按实际主控拓扑替换

      - name: rl_log_flush_thread
        type: simple_thread

      - name: joy_stick_pub_thread
        type: simple_thread
```

构建后还要确认相同配置已经进入：

```text
build/cfg/x1_cfg.yaml
```

AimRT `simple_thread` 的 `thread_sched_policy` 和 `thread_bind_cpu` 会在 Linux 上分别配置线程调度策略与 affinity。参考：

- <https://docs.aimrt.org/v0.9.3/tutorials/cfg/executor.html>

### 6.3 配置 EtherCAT IO

在 `motion_control/module/dcu_driver_module/cfg/dcu_x1.yaml` 中：

```yaml
ethercat:
  ifname: enp2s0         # 替换为实际 EtherCAT 专用网卡
  bind_cpu: 8            # 示例：与 Control 不同的 P-core
  rt_priority: 90
  enable_dc: true
  cycle_time_ns: 1000000
```

不要直接使用示例 CPU 6/8 或仓库默认 CPU 9。具体编号取决于 i7 型号、BIOS、是否开启超线程和当前内核枚举方式。

### 6.4 选择 P-core

目标主控执行：

```bash
lscpu -e=CPU,CORE,SOCKET,MAXMHZ,MINMHZ,ONLINE
```

再读取每个逻辑 CPU 的最高频率：

```bash
for cpu in /sys/devices/system/cpu/cpu[0-9]*; do
    printf '%s ' "${cpu##*/}"
    cat "$cpu/cpufreq/cpuinfo_max_freq" 2>/dev/null
done
```

选择规则：

1. 优先选择最高频率的一组 CPU，通常对应 P-core；
2. Control 与 EtherCAT 使用不同 `CORE`；
3. 每个物理 P-core 只使用一个 SMT 逻辑 CPU，另一个 sibling 保持空闲；
4. 避免 CPU 0 和主要 housekeeping CPU；
5. 不要将 Nav2、Open3D、FastLIO 或日志线程绑定到实时核；
6. IRQ affinity、CPU isolation 和是否关闭 SMT 以 `cyclictest` 与 EtherCAT 实测结果为准。

### 6.5 进程权限

#### 临时手工验证

文件 capability 适合确认问题是否确实来自权限：

```bash
cd build
sudo setcap cap_net_raw,cap_sys_nice=ep ./aimrt_main
getcap ./aimrt_main
```

预期：

```text
./aimrt_main cap_net_raw,cap_sys_nice=ep
```

`CAP_SYS_NICE` 可以修改调度策略、优先级和 affinity，其授权面比单纯“允许本线程使用实时优先级”更宽。因此它适合短期诊断，不是首选的生产权限模型。

也可能是此前通过以下任一方式启动，所以当时实际获得了实时权限：

- 使用 `sudo ./aimrt_main ...` 或以 root 身份运行；
- shell 的 `ulimit -r` 已由 PAM limits 配置为至少 `90`；
- systemd unit 已配置 `LimitRTPRIO`；
- 进程由 `chrt` 启动；
- 二进制此前已有 `CAP_SYS_NICE`，后来又被旧 `run.sh` 的 `setcap cap_net_raw=ep` 覆盖。

#### 推荐的生产配置

生产部署建议由 systemd 只授予原始网络权限，并通过资源上限允许线程设置实时优先级：

```ini
[Service]
User=f1
WorkingDirectory=/opt/f1
ExecStart=/opt/f1/aimrt_main --cfg_file_path=/opt/f1/cfg/x1_cfg.yaml
AmbientCapabilities=CAP_NET_RAW
CapabilityBoundingSet=CAP_NET_RAW
LimitRTPRIO=95
LimitMEMLOCK=infinity
```

`LimitRTPRIO=95` 允许进程把自己的线程设置到最高 FIFO 95，覆盖当前 EtherCAT 90 和建议的 Control 80，但不授予完整的 `CAP_SYS_NICE`。如还要在代码中动态修改某些环境不允许的 CPU affinity，先实测普通用户对自身线程的 affinity 是否足够，再决定是否增加 capability，不要默认扩大权限。

如果仍需从交互 shell 手工启动，可为专用用户组配置 `/etc/security/limits.d/f1-realtime.conf`：

```text
@f1rt soft rtprio 95
@f1rt hard rtprio 95
@f1rt soft memlock unlimited
@f1rt hard memlock unlimited
```

把运行用户加入 `f1rt` 后重新登录，再用 `ulimit -r` 验证。PAM limits 主要影响新的登录会话；systemd 服务应直接使用 `LimitRTPRIO`，不要假设它继承 shell 的限制。

不要用 `taskset` 给整个 `aimrt_main` 进程绑核。AimRT、ROS2 插件、EtherCAT、控制、手柄和日志均在同一进程中，进程级 affinity 会把所有线程一起限制到少数 CPU。应使用 AimRT 和 EtherCAT SDK 的线程级 affinity。

### 6.6 验证调度策略与 affinity

#### 记录权限来源

用与之前实机测试完全相同的启动方式启动程序，然后执行：

```bash
pid=$(pidof aimrt_main)
ps -o user,pid,cmd -p "$pid"
getcap ./aimrt_main
getpcaps "$pid"
ulimit -r
```

`getcap` 查看磁盘上二进制的 capability，`getpcaps` 查看运行中进程实际获得的 capability，二者不能互相替代。`ulimit -r` 必须在启动服务的同一个登录环境中检查；如果进程由 systemd 启动，则改用 `systemctl show <unit> -p LimitRTPRIO`。

以下组合基本可以判断线程没有权限切换到 FIFO，但最终仍要检查线程实际状态：

- 进程不是 root；
- `getpcaps` 没有 `cap_sys_nice`；
- `ulimit -r` 为 `0`；
- systemd 没有设置 `LimitRTPRIO`；
- 进程也不是通过 `chrt` 启动。

#### 检查每个线程的实际策略

```bash
pid=$(pidof aimrt_main)
ps -T -p "$pid" -o pid,tid,user,psr,cls,rtprio,pri,comm
```

关键字段判定：

| `CLS` / `RTPRIO` | 含义 | 当前期望 |
|---|---|---|
| `FF / 90` | `SCHED_FIFO` 优先级 90 | EtherCAT `ecat_io_loop` |
| `FF / 80` | `SCHED_FIFO` 优先级 80 | 完成改造后的 1 kHz Control executor |
| `TS / -` | 普通 `SCHED_OTHER` | 非实时辅助线程；若出现在上述两个线程则未达到目标 |

按线程名提取 EtherCAT TID 并验证：

```bash
tid=$(ps -T -p "$pid" -o tid=,comm= | awk '$2 == "ecat_io_loop" {print $1}')
chrt -p "$tid"
taskset -cp "$tid"
grep Cpus_allowed_list "/proc/$pid/task/$tid/status"
```

判断标准：

- `chrt -p` 输出 `SCHED_FIFO` 且优先级为 `90`，才算 EtherCAT 实时调度真正生效；
- `taskset -cp` 和 `Cpus_allowed_list` 应只包含为 EtherCAT 选定的 P-core 逻辑 CPU；
- Control 线程目前预计仍显示 `TS`，直到为对应 AimRT executor 增加 `SCHED_FIFO` 配置并验证；
- 修复 pthread 返回值检查前，不把“Thread set priority”之类的程序日志当作成功证据。

#### 核对内核实时能力

```bash
uname -a
cat /sys/kernel/realtime 2>/dev/null
grep PREEMPT /boot/config-"$(uname -r)" 2>/dev/null
```

`/sys/kernel/realtime` 为 `1` 通常表示 PREEMPT_RT 内核；如果文件不存在，结合 `/boot/config-*` 和内核版本判断。普通内核也能运行 `SCHED_FIFO`，但它不等价于 PREEMPT_RT，最坏时延需要由 `cyclictest` 和业务线程统计共同证明。

### 6.7 如何解释“之前单独实机运行正常”

此前的单独实机运行结果仍然有价值：它说明当时使用的二进制、网卡、EtherCAT/DCU 链路、关节映射以及基本控制路径能够工作。它不能单独证明以下内容：

- EtherCAT 线程当时确实是 `SCHED_FIFO:90`；
- 1 kHz Control 线程具备实时调度和独立绑核；
- 在完整导航负载和 IRQ 干扰下没有超周期；
- 最坏控制时延、WKC 异常和 DC 同步误差满足安全边界。

低负载时，绝大多数周期都可能接近 1 ms，偶发的数毫秒延迟也未必立刻表现为明显动作异常。EtherCAT Distributed Clocks 和循环中的绝对时间等待能改善周期行为，但不能替代 Linux 调度保证。

建议按以下方法复现实验并形成可比较记录：

1. 使用此前完全相同的二进制、配置、用户和启动命令，只运行 `motion_control`；
2. 在电机未使能或机器人可靠悬空的安全状态下，立即保存 `ps -T`、`chrt -p`、`taskset -cp`、`getpcaps`、`ulimit -r` 和内核信息；
3. 分别测试“仅 motion_control 空载”“完整 F1 软件栈”“完整栈加可控 CPU/内存/磁盘/网络压力”三种场景；
4. 每种场景至少记录 Control 周期的均值、P99、P99.9、最大值和 deadline miss 次数，同时记录 EtherCAT WKC、DC 偏差、丢包和驱动器故障；
5. 对比修复调度权限、线程绑核和 PREEMPT_RT 前后的结果，以最大时延和 deadline miss 为验收依据，不以平均周期或主观动作平滑度替代；
6. 压力测试阶段保持限速、急停可达，并先禁用执行器；只有静态验收通过后才逐级恢复使能。

当前代码尚缺完整的控制周期直方图和 deadline miss 计数，因此第 4 步的数据采集需要先完成第 10 节第 4 项的观测性改造。权限与调度状态则无需改代码即可按本节命令立即确认。

## 7. 首次部署流程

### 阶段 0：采集主控信息

```bash
uname -m
lsb_release -ds
uname -r
lscpu -e=CPU,CORE,SOCKET,MAXMHZ,MINMHZ,ONLINE
ip -br link
ip -br addr
lspci -nnk | grep -iA3 -E 'ethernet|network'
```

准入要求：

- `uname -m` 为 `x86_64`；
- 内核信息或配置确认启用 PREEMPT_RT；
- 已识别 EtherCAT 专用网卡；
- 已区分 EtherCAT 与 Livox 网络；
- 已选定两个不同的 P-core 逻辑 CPU。

### 阶段 1：构建

```bash
git checkout bb50886de7f65336cdd9da3abf035a43a057d2db
git submodule update --init --recursive

source /opt/ros/humble/setup.bash
./scripts/build.sh clean
```

包含 Livox/FastLIO/Nav2 时：

```bash
./scripts/install_livox_sdk2.sh
./scripts/build_nav.sh
```

由于 AimRT 通过 FetchContent 拉取，首次 motion_control 构建需要访问对应 Git 仓库。

### 阶段 2：配置但不使能电机

修改以下配置：

1. `build/cfg/dcu_driver_module/dcu_x1.yaml`
   - `enable_actuator: false`
   - `actuator_debug: false`
   - `ethercat.ifname`
   - `ethercat.bind_cpu`
   - DCU EtherCAT ID 和执行器/CAN ID
2. `build/cfg/x1_cfg.yaml`
   - Control executor `SCHED_FIFO:80`
   - Control P-core affinity
3. `build/cfg/control_module/rl_x1.yaml`
   - 建议显式添加 `initial_state: idle`
   - 核对关节 offset、limits、策略模型和状态转换
4. Livox 配置
   - 主控 IP 当前写死为 `192.168.1.50`
   - 雷达 IP 当前写死为 `192.168.1.195`
   - 必须根据实际网络修改

### 阶段 3：无使能总线测试

机器人吊装，急停有效，执行器保持未使能：

```bash
cd build
sudo setcap cap_net_raw,cap_sys_nice=ep ./aimrt_main
./aimrt_main --cfg_file_path=./cfg/x1_cfg.yaml
```

检查：

- EtherCAT 从站数量、ID 和状态正确；
- 无 WKC/DC 周期错误；
- 无 `setschedu`/affinity 权限错误；
- `/imu/data` 和 `/joint_states` 数据合理；
- 线程显示为预期的 `FF:90`、`FF:80` 和 CPU；
- 1 kHz 周期在压力测试下无连续 deadline miss；
- 停进程、断 ROS 命令、断 EtherCAT、触发急停的安全行为符合预期。

### 阶段 4：受控使能

仅在阶段 3 全部通过后：

1. 保持机器人吊装；
2. 将 `enable_actuator` 改为 `true`；
3. 先验证 `idle`，再进入 `zero`；
4. 单独确认关节方向、零点、限位和传动矩阵；
5. 再进入 `stand`；
6. 最后以极低 `/cmd_vel_limiter` 做行走测试；
7. 任一关节方向、IMU 坐标、时序或状态异常时立即急停并回退到未使能配置。

### 阶段 5：真机导航

必须先修复第 5 节中的 P0-1、P0-2，并完成：

- Livox 驱动话题与 IMU 频率验证；
- FastLIO `/Odometry` 与点云输出验证；
- `odom -> base_footprint`、`map -> odom` TF 唯一发布者验证；
- 2D 地图与 3D ICP 地图属于实际场地；
- 激光雷达外参、安装高度和 `body_to_footprint_z` 实测；
- `/cmd_vel` 只有一个最终限速链路发布到 `/cmd_vel_limiter`；
- 真机速度上限降至吊装和低速测试已证明安全的范围。

## 8. 验收清单

### 实时性

- [ ] 使用 PREEMPT_RT；
- [ ] EtherCAT IO 为 `SCHED_FIFO:90`；
- [ ] Control 为 `SCHED_FIFO:80`；
- [ ] 当前同步 ONNX 已限制为单线程，并随 Control 固定在同一个 P-core；
- [ ] 已分别统计推理周期、非推理周期及 `session_ptr_->Run()` 的 P99.9/最大耗时；
- [ ] EtherCAT 与 Control 位于不同物理 P-core；
- [ ] 实时核未承载日志、Nav2、Open3D 或 FastLIO；
- [ ] 完成 idle 和压力场景的最大延迟测试；
- [ ] 不存在连续追赶式循环；
- [ ] 没有周期内高频日志与文件 IO。

### 硬件安全

- [ ] 急停已验证；
- [ ] 首次总线测试使用 `enable_actuator: false`；
- [ ] DCU/驱动器命令超时行为已验证；
- [ ] EtherCAT 断线和进程异常退出行为已验证；
- [ ] 关节方向、offset、limits、DCU ID、CAN ID 均已核对；
- [ ] 吊装完成 zero/stand/低速 walk 测试。

### 导航

- [ ] 真机启动 `navigation_real.launch.py`；
- [ ] odom bridge 使用 `nav_msgs/Odometry` 真机模式；
- [ ] 系统未依赖仿真 `/clock`；
- [ ] TF 树不存在重复发布者；
- [ ] `/cmd_vel` 到 `/cmd_vel_limiter` 存在唯一限速链路；
- [ ] 真机 MPPI 速度和加速度限制已降至验证范围；
- [ ] 地图、雷达 IP 和外参属于实际设备及场地。

## 9. 本次已完成的静态验证

- Shell 脚本 `bash -n` 通过；
- 关键 Python launch/节点 `py_compile` 通过；
- `dcu_x1.yaml`、`rl_x1.yaml`、`x1_cfg.yaml` 可被 YAML 解析；
- Livox `MID360_config.json` 可被 JSON 解析；
- `git diff --check` 通过；
- 仓库附带关键动态库已确认是 Linux x86-64；
- 当前审查环境为 macOS ARM64，项目顶层 CMake 明确只支持 Linux，因此尚未在本机完成 Ubuntu x86-64 编译、PREEMPT_RT 延迟测试、EtherCAT/DCU 联机和电机使能测试。

## 10. 后续代码工作

建议按以下顺序实施：

1. 修复真机导航启动入口和真机 odom bridge；
2. 修正 pthread API 返回值检查，并把 EtherCAT 实时调度/绑核失败传播到执行器使能前；
3. 为 Control executor 增加 `SCHED_FIFO` 与 affinity 配置；
4. 增加控制循环 deadline 统计和超周期跳过；
5. 以 systemd `LimitRTPRIO` 收敛生产权限，并修复会覆盖 capability 的 `run.sh`；
6. 增加 `/joint_cmd` 新鲜度软件看门狗及可验证的失能策略；
7. 修复 `build_all.sh`、`build_nav.sh --no-livox` 和 simulation CMake 选项；
8. 降低并统一真机/仿真的速度安全边界；
9. 增加主控 preflight、systemd 服务和自动验收脚本。
