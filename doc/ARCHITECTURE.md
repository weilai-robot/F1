# F1 项目架构文档

## 概览

F1 是智元 X1 人形机器人项目，包含 **运动控制** 和 **导航避障** 两大子系统，通过 ROS2 话题在运行时解耦通信。

```
F1/
├── motion_control/          # 运动控制子系统 (CMake + AimRT)
│   ├── assistant/           # ROS2 辅助工具
│   ├── install/             # 启动脚本 + 配置文件
│   │   └── linux/bin/
│   │       ├── run.sh       # 真机入口
│   │       ├── run_sim.sh   # 仿真入口
│   │       └── cfg/         # x1_cfg.yaml / x1_cfg_sim.yaml
│   ├── module/              # 核心功能模块
│   │   ├── control_module/      # RL 策略推理 + PD 控制器 + 状态机
│   │   ├── dcu_driver_module/   # 真机 DCU 硬件通信
│   │   ├── joy_stick_module/    # 手柄遥控
│   │   ├── sim_module/          # MuJoCo 仿真
│   │   └── ankle_identifier_module/
│   ├── pkg/pkg1/            # 动态库打包 → libpkg1.so
│   └── protocols/           # 消息协议定义
│
├── navigation/              # 导航避障子系统 (ROS2 colcon)
│   ├── livox_ros_driver2/  # Livox MID-360 激光雷达驱动
│   ├── fast_lio2/          # LIO 里程计 (SLAM)
│   ├── open3d_loc/         # Open3D 3D 定位
│   ├── livox_laser_simulation_ros2/  # Gazebo LiDAR 仿真插件
│   ├── humanoid_sim/       # Gazebo 仿真环境 + Nav2 配置 + odom_bridge
│   ├── MuJoCo-LiDAR/       # MuJoCo LiDAR 传感器仿真 (pip package)
│   └── agibot_x1_train/    # RL 策略训练代码 (Isaac Gym, 独立使用)
│
├── cmake/                  # CMake 模块 (GetAimRT, NamespaceTool 等)
├── CMakeLists.txt          # 顶层 CMake (只构建 motion_control)
├── build.sh                # 构建 motion_control
├── build_nav.sh            # 构建 navigation (colcon)
└── doc/                    # 开发文档
```

---

## 子系统 A: Motion Control (`motion_control/`)

### 构建系统

| 项 | 值 |
|----|-----|
| 构建工具 | CMake + AimRT |
| 构建方式 | `./build.sh` |
| 产出 | `aimrt_main` (二进制) + `libpkg1.so` (动态库) |
| 目标平台 | Ubuntu 22.04, GCC-13, CMake ≥ 3.24 |

### 外部依赖

- **AimRT** — 机器人中间件框架（通过 cmake/GetAimRT.cmake 拉取）
- **ROS2 (rclcpp)** — 消息通信桥接
- **onnxruntime** — RL 策略推理
- **ruckig** — 轨迹规划库
- **yaml-cpp** — 配置解析
- **MuJoCo** — 仿真（仅 sim_module）

### 模块说明

| 模块 | 功能 | 输入 | 输出 |
|------|------|------|------|
| `control_module` | RL 策略 ONNX 推理 + PD 控制 + 状态机 | `/cmd_vel_limiter`, `/imu/data`, `/joint_states` | `/joint_cmd` |
| `joy_stick_module` | 手柄遥控 | 手柄输入 | `/cmd_vel`, `/walk_mode` 等 |
| `dcu_driver_module` | 真机 DCU 硬件驱动 | `/joint_cmd` | 电机指令 |
| `sim_module` | MuJoCo 物理仿真 | `/joint_cmd` | `/joint_states`, `/imu/data` |
| `ankle_identifier_module` | 踝关节并联结构辨识 | `/joint_states` | 辨识结果 |

### 入口

```bash
# 真机
cd build && ./bin/run.sh
#   → aimrt_main --cfg_file_path=./cfg/x1_cfg.yaml
#   → 加载: JoyStickModule + ControlModule + DcuDriverModule

# 仿真
cd build && ./bin/run_sim.sh
#   → aimrt_main --cfg_file_path=./cfg/x1_cfg_sim.yaml
#   → 加载: JoyStickModule + ControlModule + SimModule
```

---

## 子系统 B: Navigation (`navigation/`)

### 构建系统

| 项 | 值 |
|----|-----|
| 构建工具 | ROS2 colcon (ament_cmake) |
| 构建方式 | `./build_nav.sh` 或 `cd navigation && colcon build --symlink-install` |
| 产出 | ROS2 packages (install/) |
| 目标平台 | Ubuntu 22.04, ROS2 Humble |

### 外部依赖

- **ROS2 Humble** — 运行时框架
- **Nav2** — 导航栈 (nav2_bringup, nav2_costmap_2d, MPPI controller)
- **PCL** — 点云处理
- **Open3D** — 3D 配准定位
- **Gazebo** — 物理仿真（仿真用）
- **Livox SDK2** — MID-360 雷达驱动依赖
- **Isaac Gym** — RL 训练（仅 agibot_x1_train）

### 子包说明

| 子包 | 类型 | 功能 |
|------|------|------|
| `livox_ros_driver2` | C++ ROS2 | MID-360 雷达驱动 → `/livox/lidar` |
| `fast_lio2` | C++ ROS2 | LIO 里程计 → `/Odometry`, `/cloud_registered_body` |
| `open3d_loc` | C++ ROS2 | 3D 全局定位 (Open3D ICP) |
| `livox_laser_simulation_ros2` | C++ ROS2 | Gazebo LiDAR 仿真插件 |
| `humanoid_sim` | Python ROS2 | Gazebo 仿真环境 + Nav2 配置 + odom_bridge |
| `MuJoCo-LiDAR` | Python pip | MuJoCo LiDAR 传感器仿真 |
| `agibot_x1_train` | Python pip | RL 策略训练 (Isaac Gym, 独立使用) |

### 入口

```bash
# 仿真流程
source navigation/install/setup.bash

# 1. Gazebo 仿真环境
ros2 launch humanoid_sim simulation.launch.py

# 2. FastLIO2 建图
ros2 launch fast_lio2 mapping.launch.py

# 3. Nav2 导航
ros2 launch humanoid_sim navigation.launch.py

# 真机流程
# 1. Livox 雷达驱动
ros2 launch livox_ros_driver2 msg_MID360.launch.py    # (launch_ROS2)

# 2. FastLIO2 里程计
ros2 launch fast_lio2 mapping_real.launch.py

# 3. Nav2 导航
ros2 launch humanoid_sim navigation.launch.py
```

---

## 运行时接口（两子系统耦合点）

两套子系统完全独立构建，运行时通过 ROS2 DDS 话题通信。

```
┌─────────── Navigation (ROS2 colcon) ──────────────┐
│                                                     │
│  Livox Driver ──→ FastLIO2 ──→ /Odometry ──────┐   │
│                                    /cloud_reg..  │   │
│                       ↓                         │   │
│                 odom_bridge.py                   │   │
│                       ↓                         │   │
│  Nav2 (AMCL + MPPI + Costmap)                    │   │
│    controller_server ──→ /cmd_vel ───────────────┼───┼──┐
│                                                     │   │  │
└─────────────────────────────────────────────────────┘   │  │
                                                          │  │
                                                    ROS2 DDS │
                                                          │  │
┌─── Motion Control (CMake + AimRT) ──────────────────────┘  │
│                                                              │
│  joy_stick_module                                            │
│    └── /cmd_vel (摇杆) ──┐                                   │
│    └── /walk_mode ───────┤                                   │
│                          ↓                                   │
│  control_module                                              │
│    sub: /cmd_vel_limiter  ◄──── /cmd_vel (Nav2 或摇杆)       │
│    sub: /imu/data                                            │
│    sub: /joint_states                                        │
│    pub: /joint_cmd                                           │
│    → RL Policy (ONNX) → 电机指令                              │
│                                                              │
└──────────────────────────────────────────────────────────────┘
```

### 关键接口话题

| 方向 | 话题 | 发布方 | 订阅方 | 说明 |
|------|------|--------|--------|------|
| Nav → MC | `/cmd_vel` | Nav2 controller_server (MPPI) | control_module (via `/cmd_vel_limiter`) | 导航速度指令 |
| Joy → MC | `/cmd_vel` | joy_stick_module | control_module (via `/cmd_vel_limiter`) | 遥控速度指令 |
| Joy → MC | `/walk_mode` | joy_stick_module | control_module | 行走模式切换 |
| FastLIO → Nav | `/Odometry` | fast_lio2 | odom_bridge.py | LIO 里程计 |
| FastLIO → Nav | `/cloud_registered_body` | fast_lio2 | Nav2 Costmap (VoxelLayer) | 注册点云 |
| odom_bridge → Nav | `/odom` + TF | odom_bridge.py | Nav2 (AMCL, Costmap) | 标准 odom |

### ⚠️ Topic 对接注意事项

**`/cmd_vel` → `/cmd_vel_limiter` 命名不一致：**

- Nav2 MPPI 输出 `/cmd_vel`
- control_module 订阅 `/cmd_vel_limiter`（见 `rl_x1_sim.yaml` 第 4 行）
- 对接方案（二选一）：
  1. **Launch remap**: 在 Nav2 launch 中 remap `cmd_vel` → `cmd_vel_limiter`
  2. **配置修改**: 修改 nav2 params 中 `cmd_vel_topic` 直接指向 `/cmd_vel_limiter`

**同时使用手柄和导航时：**
- 手柄和 Nav2 都会发布 `/cmd_vel`，存在冲突
- 建议通过模式切换控制优先级：手柄 `/walk_mode` 进入行走模式后，ControlModule 选择性接收

---

## 构建命令速查

```bash
# ========== 运动控制 ==========
cd /path/to/F1
./build.sh                          # 构建 motion_control → build/

# ========== 导航 ==========
cd /path/to/F1
./build_nav.sh                      # 构建 navigation → navigation/install/

# ========== 分别 source ==========
# Motion Control 无需 source (aimrt_main 直接运行)
# Navigation 需要 source:
source navigation/install/setup.bash

# ========== 格式化 ==========
./format.sh                         # 仅格式化 motion_control 代码
```
