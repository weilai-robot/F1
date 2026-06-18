# F1 — AgiBot X1 Humanoid Robot System

English | [中文](README.zh_CN.md)

## Introduction

This project is the complete software system for the AgiBot X1 humanoid robot, comprising two subsystems that communicate at runtime via ROS2 topics:

- **Motion Control** (`motion_control/`): Built on AimRT middleware with reinforcement learning policies for locomotion control. Includes RL policy inference, PD controllers, state machines, and hardware drivers.
- **Navigation** (`navigation/`): Built on ROS2 Nav2 stack using Livox MID-360 LiDAR + FastLIO2 for SLAM, with MPPI controller for autonomous navigation and obstacle avoidance.

For detailed `AimRT` tutorials, visit the [AimRT official website](https://aimrt.org/).

![x1](doc/x1.jpg)

## Software Architecture

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
│    controller_server ──→ /cmd_vel ───────────────┼───┼─────┼──→  (Nav2 or Joystick)                │
│                                                     │     │    → RL Policy (ONNX) → Motor Cmd      │
└─────────────────────────────────────────────────────┘     └──────────────────────────────────────┘
```

For detailed architecture, see [ARCHITECTURE.md](ARCHITECTURE.md).

## Directory Structure

```bash
F1/
├── motion_control/             # Motion Control subsystem (CMake + AimRT)
│   ├── CMakeLists.txt
│   ├── assistant/             # ROS2 auxiliary tools
│   ├── install/linux/bin/     # Launch scripts & configs
│   │   ├── run.sh             #   Real robot entry
│   │   ├── run_sim.sh         #   Simulation entry
│   │   └── cfg/               #   x1_cfg.yaml / x1_cfg_sim.yaml
│   ├── module/                # Core modules
│   │   ├── control_module/        # RL policy inference + PD controller + state machine
│   │   ├── dcu_driver_module/     # Real robot DCU hardware communication
│   │   ├── joy_stick_module/      # Joystick remote control
│   │   ├── sim_module/            # MuJoCo simulation
│   │   └── ankle_identifier_module/
│   ├── pkg/                  # Shared library packaging → libpkg1.so
│   └── protocols/            # Message protocol definitions
│
├── navigation/                 # Navigation subsystem (ROS2 colcon)
│   ├── livox_ros_driver2/        # MID-360 LiDAR driver
│   ├── fast_lio2/                # LIO odometry (SLAM)
│   ├── humanoid_sim/             # Gazebo simulation + Nav2 config + odom_bridge
│   ├── open3d_loc/               # Open3D 3D global localization
│   ├── livox_laser_simulation_ros2/  # Gazebo LiDAR simulation plugin
│   ├── MuJoCo-LiDAR/             # MuJoCo LiDAR sensor simulation (pip)
│   └── agibot_x1_train/          # RL policy training code (Isaac Gym, standalone)
│
├── cmake/                     # CMake modules (GetAimRT, NamespaceTool, etc.)
├── CMakeLists.txt             # Top-level CMake (builds motion_control)
├── build.sh                   # Build motion_control
├── build_nav.sh               # Build navigation (colcon)
├── run_nav_sim.sh             # Simulation navigation one-click launch
├── run_nav_real.sh            # Real robot navigation one-click launch
├── format.sh                  # Code formatting
├── ARCHITECTURE.md            # Architecture document
├── doc/                       # Development docs
└── test.sh                    # Test scripts
```

## Prerequisites

### Common Dependencies

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

### Motion Control Extras

```bash
# Simulation dependencies
sudo apt install jstest-gtk libglfw3-dev libdart-external-lodepng-dev

# AimRT mirror source (for China)
source url_gitee.bashrc
```

- Install the Linux [realtime kernel patch](https://wiki.linuxfoundation.org/realtime/start) for real-robot debugging.

### Navigation Extras

```bash
# Nav2 navigation stack
sudo apt install -y \
    ros-humble-nav2-bringup \
    ros-humble-nav2-costmap-2d \
    ros-humble-nav2-controller \
    ros-humble-nav2-planner \
    ros-humble-tf2-tools \
    ros-humble-octomap-server

# Gazebo simulation (optional)
sudo apt install -y ros-humble-gazebo-ros-pkgs

# Livox SDK2 (required by MID-360 driver)
# See livox_ros_driver2 documentation
```

## Build

### Build Motion Control

```bash
source /opt/ros/humble/setup.bash
source url_gitee.bashrc

./build.sh $DOWNLOAD_FLAGS
```

### Build Navigation

```bash
source /opt/ros/humble/setup.bash

./build_nav.sh
```

> MuJoCo-LiDAR and agibot_x1_train are optional pip packages:
> ```bash
> cd navigation && pip install -e MuJoCo-LiDAR    # MuJoCo LiDAR simulation
> cd navigation && pip install -e agibot_x1_train  # RL training (requires Isaac Gym)
> ```

## Launch

### Entry Points

| Scenario | Motion Control | Navigation | Description |
|----------|---------------|------------|-------------|
| Sim walking (joystick) | `run_sim.sh` | — | Joystick-controlled walking |
| Real robot walking | `run.sh` | — | Joystick-controlled walking |
| Sim navigation (sim_module) | `run_sim.sh` | `run_sim_nav.sh` | Production control chain + MuJoCo LiDAR bridge |
| Sim navigation (Gazebo) | `run_sim.sh` | `run_nav_sim.sh` | Gazebo physics + Nav2 |
| Real robot navigation | `run.sh` | `run_nav_real.sh` | Navigation stack + locomotion |

### 1. Motion Control

**Simulation:** (connect joystick receiver first)

```bash
cd build/
./run_sim.sh
```

**Real Robot:** (configure library path on first run)

```bash
# First-time setup (only once)
sudo vi /etc/ld.so.conf
# Add:
#   /opt/ros/humble/lib
#   {your_project_path}/build/install/lib
sudo ldconfig

# Launch
cd build/
./run.sh
```

### 2. Navigation

**Simulation navigation (one-click, tmux multi-window):**

```bash
source navigation/install/setup.bash
./run_nav_sim.sh
# Windows: [0] Gazebo  [1] FastLIO2  [2] Nav2  [3] OctoMap
```

**Real robot navigation (one-click, tmux multi-window):**

```bash
source navigation/install/setup.bash
./run_nav_real.sh
# Windows: [0] Livox driver  [1] FastLIO2  [2] Nav2
```

### Joystick Control

For control instructions, see [Joystick Control Module](doc/joy_stick_module/joy_stick_module.md).

## License Agreement

The code in this project runs on the [AimRT](https://aimrt.org/) framework. Source code is released under the [MULAN](https://spdx.org/licenses/MulanPSL-2.0.html) license.

## Usage Instructions

If you have any questions or issues, please use `Issues`. To contribute code, fork the repository and submit a pull request.
