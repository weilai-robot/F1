# F1 — 智元 X1 人形机器人系统

## 简介

本项目是智元灵犀 X1 人形机器人的完整软件系统，包含 **运动控制** 和 **导航避障** 两大子系统，通过 ROS2 话题在运行时解耦通信。

- **运动控制** (`motion_control/`)：基于 AimRT 中间件，使用强化学习策略进行 locomotion 控制，包含 RL 策略推理、PD 控制器、状态机、硬件驱动等模块。
- **导航避障** (`navigation/`)：基于 ROS2 Nav2 导航栈，使用 Livox MID-360 激光雷达 + FastLIO2 进行 SLAM，结合 MPPI 控制器实现自主导航与避障。

关于 `AimRT` 组件的详细教程可参考 [AimRT 官方网站](https://aimrt.org/)。

![x1](doc/x1.jpg)

## 软件架构

![sw_arch](doc/sw_arch.png)

```
┌─────────── Navigation (ROS2 colcon) ──────────────┐     ┌── Motion Control (CMake + AimRT) ──┐
│                                                     │     │                                      │
│  Livox Driver ──→ FastLIO2 ──→ /Odometry ──────┐   │     │  joy_stick_module                     │
│                                    /cloud_reg..  │   │     │    └── /cmd_vel ──┐                    │
│                       ↓                         │   │     │    └── /walk_mode │                    │
│                 odom_bridge.py                   │   │     │                   ↓                    │
│                       ↓                         │   │     │  control_module                      │
│  Nav2 (AMCL + MPPI + Costmap)                    │   │     │    sub: /cmd_vel_limiter ◄── /cmd_vel │
│    controller_server ──→ /cmd_vel ───────────────┼───┼─────┼──→  (Nav2 或 摇杆)                    │
│                                                     │     │    → RL Policy (ONNX) → 电机指令       │
└─────────────────────────────────────────────────────┘     └──────────────────────────────────────┘
```

详细架构说明请参考 [ARCHITECTURE.md](ARCHITECTURE.md)。

## 目录结构

```bash
F1/
├── motion_control/             # 运动控制子系统 (CMake + AimRT)
│   ├── CMakeLists.txt
│   ├── assistant/             # ROS2 辅助工具
│   ├── install/linux/bin/     # 启动脚本与配置
│   │   ├── run.sh             #   真机入口
│   │   ├── run_sim.sh         #   仿真入口
│   │   └── cfg/               #   x1_cfg.yaml / x1_cfg_sim.yaml
│   ├── module/                # 核心模块
│   │   ├── control_module/        # RL 策略推理 + PD 控制器 + 状态机
│   │   ├── dcu_driver_module/     # 真机 DCU 硬件通信
│   │   ├── joy_stick_module/      # 手柄遥控
│   │   ├── sim_module/            # MuJoCo 仿真
│   │   └── ankle_identifier_module/
│   ├── pkg/                  # 动态库打包 → libpkg1.so
│   └── protocols/            # 消息协议定义
│
├── navigation/                 # 导航避障子系统 (ROS2 colcon)
│   ├── livox_ros_driver2/        # MID-360 激光雷达驱动
│   ├── fast_lio2/                # LIO 里程计 (SLAM)
│   ├── humanoid_sim/             # Gazebo 仿真 + Nav2 配置 + odom_bridge
│   ├── open3d_loc/               # Open3D 3D 全局定位
│   ├── livox_laser_simulation_ros2/  # Gazebo LiDAR 仿真插件
│   ├── MuJoCo-LiDAR/             # MuJoCo LiDAR 传感器仿真 (pip)
│   └── agibot_x1_train/          # RL 策略训练代码 (Isaac Gym, 独立使用)
│
├── cmake/                     # CMake 模块 (GetAimRT, NamespaceTool 等)
├── CMakeLists.txt             # 顶层 CMake (构建 motion_control)
├── build.sh                   # 构建 motion_control
├── build_nav.sh               # 构建 navigation (colcon)
├── run_nav_sim.sh             # 仿真导航一键启动
├── run_nav_real.sh            # 真机导航一键启动
├── format.sh                  # 代码格式化
├── ARCHITECTURE.md            # 架构文档
├── doc/                       # 开发文档
└── test.sh                    # 测试脚本
```

## 环境准备

### 通用依赖

- **[GCC-13](https://www.gnu.org/software/gcc/gcc-13/)**
- **[CMake](https://cmake.org/download/)** ≥ 3.24
- **[ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)**
- **[ONNX Runtime](https://github.com/microsoft/onnxruntime)**

```bash
sudo apt update
sudo apt install -y build-essential cmake git libprotobuf-dev protobuf-compiler

git clone --recursive https://github.com/microsoft/onnxruntime
cd onnxruntime
./build.sh --config Release --build_shared_lib --parallel
cd build/Linux/Release/
sudo make install
```

### 运动控制额外依赖

```bash
# 仿真环境依赖
sudo apt install jstest-gtk libglfw3-dev libdart-external-lodepng-dev

# AimRT 加速源 (国内)
source url_gitee.bashrc
```

- 若要启动实机调试，需要安装 Linux [实时内核补丁](https://wiki.linuxfoundation.org/realtime/start)。

### 导航额外依赖

```bash
# Nav2 导航栈
sudo apt install -y \
    ros-humble-nav2-bringup \
    ros-humble-nav2-costmap-2d \
    ros-humble-nav2-controller \
    ros-humble-nav2-planner \
    ros-humble-tf2-tools \
    ros-humble-octomap-server

# Gazebo 仿真 (可选)
sudo apt install -y ros-humble-gazebo-ros-pkgs

# Livox SDK2 (MID-360 驱动依赖)
# 参考 livox_ros_driver2 文档安装
```

## 构建

### 构建运动控制

```bash
source /opt/ros/humble/setup.bash
source url_gitee.bashrc

./build.sh $DOWNLOAD_FLAGS
```

### 构建导航

```bash
source /opt/ros/humble/setup.bash

./build_nav.sh
```

> MuJoCo-LiDAR 和 agibot_x1_train 为可选的 pip 包，按需手动安装：
> ```bash
> cd navigation && pip install -e MuJoCo-LiDAR    # MuJoCo LiDAR 仿真
> cd navigation && pip install -e agibot_x1_train  # RL 训练 (需 Isaac Gym)
> ```

## 启动

### 入口总览

| 场景 | 运动控制 | 导航 | 说明 |
|------|---------|------|------|
| 仿真遥控行走 | `run_sim.sh` | — | 手柄控制行走 |
| 真机遥控行走 | `run.sh` | — | 手柄控制行走 |
| 仿真导航 (sim_module) | `run_sim.sh` | `run_sim_nav.sh` | 真机一致控制链路 + MuJoCo LiDAR 桥接 |
| 仿真导航 (Gazebo) | `run_sim.sh` | `run_nav_sim.sh` | Gazebo 物理仿真 + Nav2 |
| 真机导航 + 行走 | `run.sh` | `run_nav_real.sh` | 导航栈 + 运动控制 |

### 1. 运动控制

**仿真：**（先插入手柄接收器）

```bash
cd build/
./run_sim.sh
```

**真机：**（首次需配置动态库路径）

```bash
# 首次配置 (仅需执行一次)
sudo vi /etc/ld.so.conf
# 添加:
#   /opt/ros/humble/lib
#   {你的工程绝对路径}/build/install/lib
sudo ldconfig

# 启动
cd build/
./run.sh
```

### 2. 导航

**仿真导航（一键启动，tmux 多窗口）：**

```bash
source navigation/install/setup.bash
./run_nav_sim.sh
# 窗口: [0] Gazebo  [1] FastLIO2  [2] Nav2  [3] OctoMap
```

**真机导航（一键启动，tmux 多窗口）：**

```bash
source navigation/install/setup.bash
./run_nav_real.sh
# 窗口: [0] Livox 驱动  [1] FastLIO2  [2] Nav2
```

### 手柄控制

具体控制方法请参考 [手柄控制模块](doc/joy_stick_module/joy_stick_module.zh_CN.md)。

## 许可协议

本工程提供的代码运行于 [AimRT](https://aimrt.org/) 框架之上。源代码根据 [MULAN](https://spdx.org/licenses/MulanPSL-2.0.html) 许可协议发布。

## 使用说明

如果您对该仓库有任何疑问或问题，请使用 `Issues`。如需贡献代码，请 fork 该仓库并提交 pull request。
