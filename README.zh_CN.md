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
│   ├── run_sim_nav.sh          #   一键启动仿真导航 (6 个 tmux 窗口)
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

```bash
# 构建 motion_control
scripts/build.sh

# 构建 navigation
scripts/build_nav.sh

# 一键构建全部
scripts/build_all.sh
```

### 4. 运行

| 场景 | 命令 |
|------|------|
| 仿真行走（仅控制） | `cd build/ && ./run_sim.sh` |
| 真机行走（仅控制） | `cd build/ && sudo setcap cap_net_raw=ep ./aimrt_main && ./run.sh` |
| 仿真全栈导航 | `scripts/run_sim_nav.sh` 然后 `scripts/send_nav_goal.sh 5.0 0.0` |
| 真机自主导航 | `cd build/ && ./run.sh` 然后 `scripts/run_nav_real.sh` 然后 `scripts/send_nav_goal.sh 3.0 0.0` |

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
