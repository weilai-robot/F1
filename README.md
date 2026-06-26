# F1 — AgiBot X1 Humanoid Robot System

English | [中文](README.zh_CN.md)

## Introduction

This project is the complete software system for the AgiBot X1 humanoid robot, comprising two subsystems that communicate at runtime via ROS2 topics:

- **Motion Control** (`motion_control/`): Built on AimRT middleware with reinforcement learning policies for locomotion control. Includes RL policy inference, PD controllers, state machines, and hardware drivers.
- **Navigation** (`navigation/`): Built on ROS2 Nav2 stack using Livox MID-360 LiDAR + FastLIO2 for SLAM, with MPPI controller for autonomous navigation and obstacle avoidance.

> **Repository Architecture**: This is a **thin integration repo** — `navigation/` and `motion_control/` are [Git submodules](doc/SUBMODULE_GUIDE.md), independently developed and version-locked here. Clone with `--recursive` to get everything.

| Submodule | Repository | Build System |
|-----------|-----------|--------------|
| `navigation/` | [weilai-robot/Humanoid_navigation](https://github.com/weilai-robot/Humanoid_navigation) | ROS2 colcon |
| `motion_control/` | [weilai-robot/Humanoid_motion](https://github.com/weilai-robot/Humanoid_motion) | CMake + AimRT |

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
F1/                              # Thin integration repo (this repo)
├── motion_control/             # [submodule] Motion Control subsystem (CMake + AimRT)
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
├── navigation/                 # [submodule] Navigation subsystem (ROS2 colcon)
│   ├── livox_ros_driver2/        # MID-360 LiDAR driver
│   ├── fast_lio2/                # LIO odometry (SLAM)
│   ├── humanoid_sim/             # Gazebo simulation + Nav2 config + odom_bridge
│   ├── open3d_loc/               # Open3D 3D global localization
│   ├── livox_laser_simulation_ros2/  # Gazebo LiDAR simulation plugin
│   ├── MuJoCo-LiDAR/             # MuJoCo LiDAR sensor simulation (pip)
│   └── agibot_x1_train/          # RL policy training code (Isaac Gym, standalone)
│
├── cmake/                     # CMake modules (GetAimRT, NamespaceTool, etc.) — integration repo
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

## Quick Start (Clone with Submodules)

```bash
git clone --recursive https://github.com/weilai-robot/F1.git
```

If already cloned without `--recursive`:
```bash
cd F1
git submodule update --init --recursive
```

> 📖 **New to submodules?** Read the [Submodule Guide](doc/SUBMODULE_GUIDE.md) for daily workflows, branching strategy, and common pitfalls.

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

## How to Use

The F1 system supports four deployment scenarios. Each scenario has its own build and launch steps.

> **ROS2 source:** All scripts auto-detect `ROS_SETUP_BASH` (conda, `/opt/ros`, or `AMENT_PREFIX_PATH`). To override: `export ROS_SETUP_BASH=/your/ros/setup.bash`

---

### Scenario 1 — Motion Control Simulation (Walking only, no navigation)

Use this when testing RL locomotion policies, gait tuning, or joystick control in MuJoCo.

```bash
# 1. Build motion_control (if not already built)
./build.sh

# 2. Launch sim (opens MuJoCo viewer + joystick control)
cd build/
./run_sim.sh                # loads x1_cfg_sim.yaml, joystick input
```

**Joystick operations:** Stand → press `walk_mode` button to start walking → use joystick to steer. See [Joystick docs](doc/joy_stick_module/joy_stick_module.md).

---

### Scenario 2 — Motion Control Real Robot (Walking only, no navigation)

Use this for deploying the RL policy on the physical X1 robot with joystick teleoperation.

```bash
# 1. Build motion_control
./build.sh

# 2. One-time: configure shared library path
sudo vi /etc/ld.so.conf
#   Add these lines:
#     /opt/ros/humble/lib
#     {your_project_path}/build/install/lib
sudo ldconfig

# 3. Launch real robot control
cd build/
sudo setcap cap_net_raw=ep ./aimrt_main    # grant raw socket for DCU
./run.sh                                     # loads x1_cfg.yaml, connects to real hardware
```

**Requirements:** Realtime kernel patch, DCU module connected, joystick receiver paired.

---

### Scenario 3 — Navigation Simulation (Nav2 + MuJoCo, no real hardware)

Use this to test the full navigation stack (SLAM + path planning + obstacle avoidance) against a simulated lab environment. Motion control runs inside the same sim.

```bash
# 1. Build motion_control (provides aimrt_main + MuJoCo physics)
./build.sh

# 2. Build navigation stack (FastLIO2 + Nav2 + humanoid_sim)
./build_nav.sh

# 3. One-time: compile libruckig from source if GLIBC < 2.32
./build_ruckig.sh && rm -rf build/ && ./build.sh

# 4. Launch full sim navigation pipeline (tmux, 6 windows)
./run_sim_nav.sh
#    [0] aimrt_main      — ONNX RL policy + MuJoCo physics simulation
#    [1] lidar_bridge    — MuJoCo LiDAR ray-tracing + /clock publisher
#    [2] fastlio         — FastLIO2 SLAM odometry
#    [3] nav2            — Nav2 stack (MPPI + Costmap + Planner) + RViz
#    [4] octomap         — OctoMap 3D mapping (optional)
#    [5] record          — Data recording (manual start)

# 5. In another terminal, send a navigation goal
./send_nav_goal.sh 5.0 0.0          # navigate to (5.0, 0.0)
#    or: ./send_nav_goal.sh --walk-only   # just enter walking mode
#    or: ./send_nav_goal.sh --batch       # run automated test scenarios

# 6. (Optional) Check SLAM drift
python3 drift_check.py 5.0 0.0
```

**RViz shows:** Static map, global/local costmap, global path (green), MPPI local path (yellow), goal pose (red arrow), registered point cloud.

---

### Scenario 4 — Navigation + Motion Control Real Robot (Full autonomous navigation)

Use this to run autonomous navigation on the physical X1 robot. The navigation stack provides velocity commands; the RL policy converts them to joint-level motor commands.

```bash
# 1. Build both subsystems
./build.sh
./build_nav.sh

# 2. Configure real robot library path (one-time, see Scenario 2)

# 3. Start motion control (robot stands up, waits for walk_mode)
cd build/
sudo setcap cap_net_raw=ep ./aimrt_main
./run.sh

# 4. In another terminal, start the navigation stack
./run_nav_real.sh                 # or: ./run_nav_real.sh --no-rviz
#    [0] Livox driver   — MID-360 LiDAR data acquisition
#    [1] FastLIO2       — Real robot LIO odometry
#    [2] Nav2            — Navigation stack + odom_bridge + RViz

# 5. In a third terminal, send navigation goal (also triggers walk_mode)
./send_nav_goal.sh 3.0 0.0
#    or publish goal via RViz "2D Goal Pose" tool
```

**Important:** The robot must be standing and stable before `run_nav_real.sh` is started. Use the joystick to confirm the robot has entered `stand` mode first.

---

### Quick Reference

| Scenario | Build | Launch Command | Joystick Required |
|----------|-------|---------------|:-:|
| 1. Sim walking | `./build.sh` | `cd build/ && ./run_sim.sh` | ✓ |
| 2. Real walking | `./build.sh` | `cd build/ && ./run.sh` | ✓ |
| 3. Sim navigation | `./build.sh && ./build_nav.sh` | `./run_sim_nav.sh` + `./send_nav_goal.sh` | ✗ |
| 4. Real navigation | `./build.sh && ./build_nav.sh` | `./run.sh` + `./run_nav_real.sh` + `./send_nav_goal.sh` | ✓ (stand-up) |

## License Agreement

The code in this project runs on the [AimRT](https://aimrt.org/) framework. Source code is released under the [MULAN](https://spdx.org/licenses/MulanPSL-2.0.html) license.

## Usage Instructions

If you have any questions or issues, please use `Issues`. To contribute code, fork the repository and submit a pull request.
