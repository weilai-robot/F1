# F1 — Humanoid Robot System (Integration Repo)

[中文](README.zh_CN.md) | English

## What is this repo?

F1 is a **thin integration repository** for the X1 humanoid robot. It does NOT contain subsystem code directly — instead, it wires together two independently developed submodules via Git and provides unified build/run scripts and Docker definitions.

| Submodule | Repository | Build System | Team |
|-----------|-----------|--------------|------|
| `navigation/` | [weilai-robot/Humanoid_navigation](https://github.com/weilai-robot/Humanoid_navigation) | ROS2 colcon | Navigation |
| `motion_control/` | [weilai-robot/Humanoid_motion](https://github.com/weilai-robot/Humanoid_motion) | CMake + AimRT | Control |

The two subsystems communicate at runtime via ROS2 topics — there is zero code-level dependency between them.

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

> Full architecture doc: [doc/ARCHITECTURE.md](doc/ARCHITECTURE.md)

## Directory Structure

```
F1/                              # This repo — thin integration layer
├── navigation/                  # [submodule] → Humanoid_navigation
├── motion_control/              # [submodule] → Humanoid_motion
├── scripts/                     # Unified build & run scripts
│   ├── build_all.sh            #   Build both subsystems
│   ├── build.sh                #   Build motion_control only
│   ├── build_nav.sh            #   Build navigation only
│   ├── run_sim_nav.sh          #   Launch full sim navigation (6 tmux panes)
│   ├── run_nav_real.sh         #   Launch real robot navigation
│   ├── send_nav_goal.sh        #   Send navigation goal + trigger walk mode
│   ├── drift_check.py          #   SLAM drift diagnostics
│   ├── axis_check.py           #   Coordinate axis alignment diagnostics
│   ├── nav_test_runner.py      #   Automated navigation test scenarios
│   └── ...
├── cmake/                       # Shared CMake modules (GetAimRT, NamespaceTool)
├── CMakeLists.txt              # Top-level CMake entry (builds motion_control)
├── docker/                     # Docker environment definitions
├── Dockerfile                  # Multi-stage build: motion_control + navigation
├── docker-compose.yml          # Sim navigation container
├── doc/                        # Architecture, submodule guide, API docs
└── README.md
```

---

## Quick Start

### 1. Clone

```bash
git clone --recursive https://github.com/weilai-robot/F1.git
cd F1
```

> Already cloned without `--recursive`? Run:
> ```bash
> git submodule update --init --recursive
> ```

### 2. Prerequisites

- **GCC-13** / **CMake** ≥ 3.24
- **[ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)**
- **[ONNX Runtime](https://github.com/microsoft/onnxruntime)** (build from source)

```bash
sudo apt update
sudo apt install -y build-essential cmake git libprotobuf-dev protobuf-complier
```

**Motion control extras:**
```bash
sudo apt install jstest-gtk libglfw3-dev libdart-external-lodepng-dev
```

**Navigation extras:**
```bash
sudo apt install -y \
    ros-humble-nav2-bringup ros-humble-nav2-costmap-2d \
    ros-humble-nav2-controller ros-humble-nav2-planner \
    ros-humble-tf2-tools ros-humble-octomap-server
```

### 3. Build

```bash
# Build motion_control
scripts/build.sh

# Build navigation
scripts/build_nav.sh

# Or build both at once
scripts/build_all.sh
```

### 4. Run

| Scenario | Command |
|----------|---------|
| Sim walking only | `cd build/ && ./run_sim.sh` |
| Real robot walking | `cd build/ && sudo setcap cap_net_raw=ep ./aimrt_main && ./run.sh` |
| Full sim navigation | `scripts/run_sim_nav.sh` then `scripts/send_nav_goal.sh 5.0 0.0` |
| Real robot navigation | `cd build/ && ./run.sh` then `scripts/run_nav_real.sh` then `scripts/send_nav_goal.sh 3.0 0.0` |

---

## Submodule Workflow

This repo uses Git submodules to pin exact versions of each subsystem. Below is the essential workflow — **read [doc/SUBMODULE_GUIDE.md](doc/SUBMODULE_GUIDE.md) for the full reference**.

### Pull latest code (including submodule updates)

```bash
git pull origin main
git submodule update --init --recursive
```

### Develop inside a submodule

```bash
cd navigation/
git checkout devel          # ⚠️ Always switch branch first (avoid detached HEAD)
# ... code, test, commit ...
git push origin devel
```

### Pin a new submodule version in this repo

After pushing changes in a submodule:

```bash
cd /path/to/F1
git add navigation/         # or motion_control/
git commit -m "chore: bump navigation to latest"
git push origin main
```

### Rollback one submodule without affecting the other

```bash
cd navigation/
git checkout <old-commit-hash>
cd ..
git add navigation/
git commit -m "revert: rollback navigation to <version>"
```

> This is the core benefit of the submodule architecture — each subsystem can be versioned and rolled back independently.

---

## Docker (Sim Navigation)

```bash
docker compose up --build
# Container starts Xvfb + full nav stack, exposes RViz via X11 or web
```

See [docker/DOCKER_GUIDE.md](docker/DOCKER_GUIDE.md) for details.

## License

Source code is released under the [MULAN](https://spdx.org/licenses/MulanPSL-2.0.html) license. The project runs on the [AimRT](https://aimrt.org/) framework.
