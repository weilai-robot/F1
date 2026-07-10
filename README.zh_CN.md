# F1 — 人形机器人系统（薄集成仓库）

[English](README.md) | 中文

## 这个仓库是什么？

F1 是一个**薄集成仓库**，用于 X1 人形机器人。它**不直接包含子系统代码**——而是通过 Git Submodule 将两个独立开发的子仓库组合在一起，并提供统一的构建脚本和 Docker 环境。

| 子模块 | 仓库地址 | 构建系统 | 团队 |
|--------|---------|---------|------|
| `navigation/` | [weilai-robot/Humanoid_navigation](https://github.com/weilai-robot/Humanoid_navigation) | ROS2 colcon | 导航团队 |
| `motion_control/` | [weilai-robot/Humanoid_motion](https://github.com/weilai-robot/Humanoid_motion) | CMake + AimRT | 控制团队 |

两个子系统在运行时通过 ROS2 话题通信，代码层面**零依赖**。

![x1](doc/x1.jpg)

## 软件架构

![sw_arch](doc/sw_arch.png)

```
┌─────────── 导航子系统 (ROS2 colcon) ──────────────┐     ┌── 运动控制子系统 (CMake + AimRT) ──┐
│                                                     │     │                                      │
│  Livox Driver ──→ FastLIO2 ──→ /Odometry ──────┐   │     │  joy_stick_module                     │
│                                    /cloud_reg..  │   │     │    └── /cmd_vel ──┐                    │
│                       ↓                         │   │     │    └── /walk_mode │                    │
│                 odom_bridge.py                   │   │     │                   ↓                    │
│                       ↓                         │   │     │  control_module                      │
│  Nav2 (AMCL + MPPI + Costmap)                    │   │     │    sub: /cmd_vel_limiter ◄── /cmd_vel │
│    controller_server ──→ /cmd_vel ───────────────┼───┼─────┼──→  (Nav2 或手柄)                     │
│                                                     │     │    → RL 策略 (ONNX) → 电机命令         │
└─────────────────────────────────────────────────────┘     └──────────────────────────────────────┘
```

> 完整架构文档：[doc/ARCHITECTURE.md](doc/ARCHITECTURE.md)

## 目录结构

```
F1/                              # 本仓库 — 薄集成层
├── navigation/                  # [submodule] → Humanoid_navigation
├── motion_control/              # [submodule] → Humanoid_motion
├── scripts/                     # 统一构建与运行脚本
│   ├── build_all.sh            #   构建两个子系统
│   ├── build.sh                #   仅构建 motion_control
│   ├── build_nav.sh            #   仅构建 navigation
│   ├── run_mujoco_nav.sh       #   MuJoCo+sim_module 联合仿真导航 (6 个 tmux 窗口)
│   ├── run_nav_real.sh         #   真机导航一键启动
│   ├── send_nav_goal.sh        #   发送导航目标 + 切换行走模式
│   ├── drift_check.py          #   SLAM 漂移诊断
│   ├── axis_check.py           #   坐标轴对齐诊断
│   ├── nav_test_runner.py      #   自动化导航测试
│   └── ...
├── cmake/                       # 公共 CMake 模块 (GetAimRT, NamespaceTool)
├── CMakeLists.txt              # 顶层 CMake 入口 (构建 motion_control)
├── docker/                     # Docker 环境定义
├── Dockerfile                  # 多阶段构建: motion_control + navigation
├── docker-compose.yml          # 仿真导航容器编排
├── doc/                        # 架构文档、Submodule 指南、API 文档
└── README.md
```

---

## 快速开始

### 1. 克隆

```bash
git clone --recursive https://github.com/weilai-robot/F1.git
cd F1
```

> 如果已经 clone 了但忘了加 `--recursive`：
> ```bash
> git submodule update --init --recursive
> ```

### 2. 环境依赖

- **GCC-13** / **CMake** ≥ 3.24
- **[ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)**
- **[ONNX Runtime](https://github.com/microsoft/onnxruntime)**（需从源码编译）

```bash
sudo apt update
sudo apt install -y build-essential cmake git libprotobuf-dev protobuf-compiler
```

**运动控制额外依赖：**
```bash
sudo apt install jstest-gtk libglfw3-dev libdart-external-lodepng-dev
```

**导航额外依赖：**
```bash
sudo apt install -y \
    ros-humble-nav2-bringup ros-humble-nav2-costmap-2d \
    ros-humble-nav2-controller ros-humble-nav2-planner \
    ros-humble-tf2-tools ros-humble-octomap-server
```

### 3. 构建

#### motion_control（运动控制）

```bash
# Release 增量构建（默认）
./scripts/build.sh

# 清理后全新构建
./scripts/build.sh clean

# Debug 构建
./scripts/build.sh Debug
```

构建产物位于 `build/`：

| 产物 | 说明 |
|------|------|
| `aimrt_main` | AimRT 主二进制 |
| `libpkg1.so` | 注册 5 个模块的共享库 |
| `cfg/x1_cfg.yaml` | 真机顶层配置 |
| `cfg/control_module/rl_x1.yaml` | 真机控制配置（频率、关节、ONNX 策略） |
| `cfg/control_module/policy/*.onnx` | RL 策略模型 |
| `cfg/dcu_driver_module/dcu_x1.yaml` | DCU 硬件驱动配置（EtherCAT、执行器、传动） |
| `run.sh` / `run_with_recording.sh` | 真机运行脚本 |

脚本会在编译后自动校验上述产物是否齐全。

#### navigation（导航）

```bash
./scripts/build_nav.sh
```

#### 一键构建全部

```bash
./scripts/build_all.sh
```

---

### 4. 真机部署运行 motion_control

> 以下为**仅在真机上部署运行运动控制**的完整步骤。

#### 4.1 前置检查

| 检查项 | 配置文件 | 当前值 | 说明 |
|--------|---------|--------|------|
| EtherCAT 网卡名 | `cfg/dcu_driver_module/dcu_x1.yaml` → `ethercat.ifname` | `enp2s0` | ⚠️ 必须与真机实际网卡名一致 |
| DCU EtherCAT ID | `dcu_x1.yaml` → `dcu_network[].ecat_id` | body=1, hip=2 | 按实际链路顺序 |
| IMU 来源 | `dcu_x1.yaml` → `imu_dcu_name` | `hip` | 使用下肢 DCU 的 IMU |
| 控制频率 | `cfg/control_module/rl_x1.yaml` → `control_frequecy` | `1000` Hz | MainLoop 1 ms 周期 |
| EtherCAT 周期 | `dcu_x1.yaml` → `ethercat.cycle_time_ns` | `1000000` ns (1 ms) | 与控制频率匹配 |
| 关节偏移量 | `rl_x1.yaml` → `joint_offset` | 全 `0.0` | 按实际标零情况调整 |

#### 4.2 启动控制

```bash
cd build

# 1. 赋予 raw socket 权限（EtherCAT 通信需要，每次重新拷贝二进制后需重做）
sudo setcap cap_net_raw=ep ./aimrt_main

# 2. 启动（方式 A：仅控制）
./aimrt_main --cfg_file_path=./cfg/x1_cfg.yaml
# 或等价脚本
bash run.sh

# 2. 启动（方式 B：控制 + ROS2 bag 录制）
bash run_with_recording.sh
```

`x1_cfg.yaml` 加载的模块：

| 模块 | 职责 |
|------|------|
| `JoyStickModule` | 遥控器/Nav2 → `/cmd_vel` 速度指令转换 |
| `ControlModule` | 状态机调度 + RL/PD 控制 + 数据日志采集 |
| `DcuDriverModule` | EtherCAT 硬件驱动（读取 IMU/关节状态，下发电机指令） |

> 真机配置**不含** `SimModule`（仿真专用）。真机依赖 DCU 驱动提供 `/imu/data`、`/joint_states`，并接收 `/joint_cmd`。

#### 4.3 状态机操作

机器人启动后进入 `initial_state`（由 `rl_x1.yaml` 指定）。通过 ROS2 话题触发状态切换：

```bash
# —— 标准上电流程 ——
ros2 topic pub --once /idle_mode  std_msgs/msg/Float32 '{data: 0.0}'   # 空闲（上电不使能）
ros2 topic pub --once /zero_mode  std_msgs/msg/Float32 '{data: 0.0}'   # 归零（使能 + 回零位）
ros2 topic pub --once /stand_mode std_msgs/msg/Float32 '{data: 0.0}'   # 站立

# —— 行走（从 stand 进入）——
ros2 topic pub --once /walk_mode  std_msgs/msg/Float32 '{data: 0.0}'   # walk_leg（纯腿部）
ros2 topic pub --once /walk_mode2 std_msgs/msg/Float32 '{data: 0.0}'   # walk_leg_arm（腿 + 肩）

# —— 速度指令（行走状态下）——
ros2 topic pub /cmd_vel_limiter geometry_msgs/msg/Twist '{linear: {x: 0.2}, angular: {z: 0.0}}'
```

状态转移规则见 `rl_x1.yaml` → `robot_states`，非法转移会被状态机拒绝。

#### 4.4 数据日志

控制模块内置数据采集系统，**在进入 `walk_leg` / `walk_leg_arm` 状态时自动触发**，无需额外操作。

| 日志 | 格式 | 路径 | 采集频率 | 单次时长 |
|------|------|------|---------|---------|
| `walk_diag_<时间戳>.csv` | CSV 文本 | `test_logs/data_csv/` | 100 Hz | 10 s (1000 帧) |
| `tm_obs_input_<时间戳>.bin` | float 二进制原始流 | `test_logs/data_csv/t_m/` | 100 Hz | 10 s (1000 帧) |

- **walk_diag**：每帧记录时间戳、步态相位、速度指令、欧拉角、角速度、各关节 action/pos/vel/effort/PD 目标值（原始 + 滤波）、IMU 四元数/陀螺/加速度。
- **tm_obs_input**：ONNX 策略网络的完整观测向量，用于离线回放。

采集满 1000 帧或离开 walk 状态 500 ms 无新帧后自动关闭文件。日志写入路径相对于进程 CWD（即 `build/test_logs/`）。

---

### 5. 仿真运行

| 场景 | 命令 |
|------|------|
| 仿真行走（仅控制） | `cd build && ./run_sim.sh` |
| 仿真全栈导航 | `./scripts/run_mujoco_nav.sh` 然后 `./scripts/send_nav_goal.sh 5.0 0.0` |
| 真机自主导航 | 先按 §4 启动控制，再 `./scripts/run_nav_real.sh`，然后 `./scripts/send_nav_goal.sh 3.0 0.0` |

---

## Submodule 工作流

本仓库通过 Git Submodule 锁定各子系统的精确版本。以下是核心操作——**完整手册请阅读 [doc/SUBMODULE_GUIDE.md](doc/SUBMODULE_GUIDE.md)**。

### 拉取最新代码（含子模块更新）

```bash
git pull origin main
git submodule update --init --recursive
```

### 在子模块中开发

```bash
cd navigation/
git checkout devel          # ⚠️ 必须先切分支（避免 detached HEAD）
# ... 写代码、测试、提交 ...
git push origin devel
```

### 在集成仓库中锁定子模块新版本

子模块推送后，回集成仓库提交一次：

```bash
cd /path/to/F1
git add navigation/         # 或 motion_control/
git commit -m "chore: bump navigation to latest"
git push origin main
```

### 单独回退某个子模块（不影响另一个）

```bash
cd navigation/
git checkout <旧commit-hash>
cd ..
git add navigation/
git commit -m "revert: rollback navigation to <version>"
```

> 这就是 submodule 架构的核心优势——每个子系统可独立版本管理和回退。

---

## Docker（仿真导航）

```bash
docker compose up --build
# 容器自动启动 Xvfb + 全栈导航，通过 X11 或 Web 暴露 RViz
```

详见 [docker/DOCKER_GUIDE.md](docker/DOCKER_GUIDE.md)。

## License

代码基于 [MULAN](https://spdx.org/licenses/MulanPSL-2.0.html) 协议发布，运行于 [AimRT](https://aimrt.org/) 框架。
