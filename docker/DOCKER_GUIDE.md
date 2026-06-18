# F1 Docker 部署指南

## 环境要求

| 项目 | 最低要求 | 推荐 |
|------|---------|------|
| OS | Ubuntu 22.04 | Ubuntu 22.04 LTS Server |
| Docker | 24.0+ | 最新稳定版 |
| CPU | 8 核 | 16 核+ |
| 内存 | 16 GB | 32 GB |
| 磁盘 | 30 GB (镜像+缓存) | 50 GB+ |
| GPU | 不需要 | 不需要 (纯 CPU 仿真) |
| 显示器 | 无头可用 (Xvfb) | 有屏最佳 (MuJoCo viewer) |

## 快速开始

### 1. 构建镜像 (~20-30 分钟首次构建)

```bash
cd /path/to/F1
docker-compose build
```

或直接用 docker：
```bash
docker build -t f1-nav-sim .
```

### 2. 进入容器

**有显示器（服务器有屏幕或 X11 forwarding）:**
```bash
# 允许 Docker 访问 X server
xhost +local:docker

# 启动
docker-compose run f1
```

**无头服务器（自动启用 Xvfb）:**
```bash
docker-compose run f1
```

### 3. 容器内运行

```bash
# 容器内, 进入 workspace
cd /workspace

# 纯行走仿真
./build/run_sim.sh

# 导航仿真 (5 个 tmux 窗口)
./run_sim_nav.sh

# 跑导航测试
python3 nav_test_runner.py --scenario A_straight_5m
python3 nav_test_runner.py --batch
```

## 常用操作

### 在另一终端进入运行中的容器

```bash
docker exec -it f1-nav-sim bash
```

### 查看 CPU/内存使用

```bash
docker stats f1-nav-sim
```

### 录制 rosbag 到持久化目录

```bash
# 容器内 — reports 目录会映射到宿主机
cd /workspace/reports
ros2 bag record /mujoco/ground_truth /cmd_vel_limiter /Odometry -o test_run_001
```

### 修改代码后重新构建

```bash
# 仅 motion_control 改动 → 重新 build.sh (容器内)
cd /workspace/src
./build.sh

# 仅 navigation 改动 → 重新 build_nav.sh (容器内)
./build_nav.sh

# 需要完整重建 (AimRT 等依赖变更)
docker-compose build --no-cache
```

## 构建缓存优化

Dockerfile 是多阶段构建，重型依赖分层缓存：

```
Stage 1 (builder):
  apt install          ← 几乎不变, 命中缓存
  pip install          ← 几乎不变, 命中缓存
  COPY 源码            ← 每次变 (代码改动)
  cmake build          ← AimRT 首次 ~15min, 后续如 cmake 缓存命中可跳过
  colcon build         ← ~3min
  pip install MuJoCo-LiDAR ← ~30s

Stage 2 (runtime):
  apt install runtime  ← 几乎不变, 命中缓存
  COPY from builder    ← 每次变
```

**首次构建 ~30min，后续代码改动重建 ~5min**（如果只改了 motion_control 的 .cc 文件，cmake 增量编译更快）。

## DDS 网络配置

容器使用 `network_mode: host`，ROS2 DDS 直接走宿主机网络。

- **单容器**：不需要额外配置，ROS2 节点间通信正常
- **多容器**：如果未来 motion_control 和 navigation 分容器部署，需要：
  ```yaml
  environment:
    - ROS_DOMAIN_ID=0
    - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # 比 FastRTPS 更稳定
  ```

## MuJoCo Viewer GUI

| 场景 | 方案 | 说明 |
|------|------|------|
| 有屏服务器 | X11 forwarding | 容器内 MuJoCo viewer 直接显示在宿主机屏幕 |
| 无头服务器 | Xvfb | 自动启动虚拟显示，viewer 不可见但物理仿真正常运行 |
| 远程访问 | VNC (可选) | 在容器内装 x11vnc + xfce，通过浏览器远程查看 |

无头模式下，MuJoCo viewer 仍然运行（渲染到 Xvfb），只是看不到画面。物理仿真和所有数据流完全正常。

## 已知问题与解决方案

### 1. Rosdep 初始化失败

Dockerfile 中跳过了 `rosdep init/update`。如果某些 ROS2 包找不到依赖：

```bash
# 容器内手动执行
sudo rosdep init
rosdep update
```

### 2. Livox SDK2 缺失

仿真不需要 Livox SDK2（仿真 LiDAR 由 MuJoCo-LiDAR 提供点云）。但 `livox_ros_driver2` 的 colcon 编译可能报依赖缺失。

如果编译失败，注释掉 `build_nav.sh` 中的 `livox_ros_driver2`：
```bash
colcon build --symlink-install \
    --packages-select \
        fast_lio \
        open3d_loc \
        ros2_livox_simulation \
        humanoid_sim
```

### 3. MuJoCo 版本冲突

| 来源 | 版本 | 用途 |
|------|------|------|
| sim_module/third_party/lib/ | 3.1.3 | C++ 物理 (bundled, 不受 pip 影响) |
| pip install | ≥3.2.0 | Python LiDAR bridge |

两者在运行时互不干扰：C++ 用 bundled .so（通过 `target_link_directories` 指定），Python 用 pip 安装的版本。

### 4. ONNX Runtime

ONNX Runtime 1.19.2 随仓库一起打入镜像（`third_party/lib/`），不需要额外安装。

## 文件映射

| 容器内路径 | 宿主机路径 | 说明 |
|-----------|-----------|------|
| `/workspace/reports/` | `./reports/` | 测试报告 (持久化) |
| `/workspace/build/log/` | `./docker/logs/` | AimRT 日志 (持久化) |
| `/tmp/.X11-unix/` | `/tmp/.X11-unix/` | X11 socket (共享) |

## 从仿真到真机

Docker 镜像用于**仿真开发和测试**。真机部署建议直接在 Ubuntu 22.04 上安装（不使用 Docker），因为：

1. 真机需要访问硬件设备（USB 手柄、Livox 雷达、DCU 以太网）
2. 真机需要实时内核（PREEMPT_RT patch），Docker 内不方便打
3. 真机需要 `setcap cap_net_raw`（DCU 通信），容器内需要 `--privileged`

**推荐流程**：仿真在 Docker 中验证 → 真机在裸机上部署。
