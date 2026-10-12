# AMP 仿真服务器运行

分支：F1 和 motion_control 均为 `feat/amp-deploy`。本次修正两个控制器配置中的
AMP 策略关节顺序，以及便携 rollout 的错误回退顺序；去掉 AMP 同状态重入，避免长按 Guide 清空历史。
ONNX 文件无需重新导出。

## 1. 更新与构建

在已有 ROS2 Humble、CMake 和项目依赖的 Linux 服务器上执行：

```bash
cd ~/code/F1
git switch feat/amp-deploy
git pull --ff-only origin feat/amp-deploy
git submodule update --init --recursive
bash scripts/build.sh
```

必须执行子模块更新和构建，使新 YAML 安装到 `build/cfg/`。
子模块停在父仓库记录的提交上即可，不必单独 checkout/pull。
构建脚本会检查 `run_sim.sh`、仿真总配置和仿真控制器配置是否存在。

ROS2 自动探测失败时，先设置实际安装路径，再重新构建：

```bash
export ROS2_SETUP_PATH=/实际路径/setup.bash
```

首次验证可核对模型文件：

```bash
sha256sum build/cfg/control_module/policy/x1_amp_soup59c_50hz.onnx
```

应为 `3e09b76f1e38085a2093abccc1e6f80c86751c9758f7f21d0f497163c99ebd00`。

## 2. 启动 run_sim

终端 A：

```bash
cd ~/code/F1/build
bash run_sim.sh
```

脚本读取 `cfg/x1_cfg_sim.yaml`，加载 `ControlModule + SimModule + JoyStickModule`，
不加载真机 DCU。控制器读取 `cfg/control_module/rl_x1_sim.yaml`，初始模式为 `stand`。
等站姿和反馈稳定、启动至少 1 秒后再切 AMP。

有桌面时使用已有 DISPLAY。SSH 无显示时，脚本会自动启动 Xvfb；若尚未安装：

```bash
sudo apt-get install -y xvfb
```

## 3. 切入 AMP

终端 B（与终端 A 使用相同 ROS_DOMAIN_ID/ROS2 环境）：

```bash
cd ~/code/F1
source scripts/ros2_source.sh
source build/install/share/ros2_plugin_proto/local_setup.bash
ros2 topic pub --once /amp_mode std_msgs/msg/Float32 '{data: 1.0}'
```

应看到 `stand -> walk_amp`。也可单次按 Guide；`/walk_mode` 对应旧腿部策略，不能用于本次 AMP 验证。
`settle_ticks=0`，切入后策略立即接管，没有 1 秒静持/软启动。

**默认仿真配置有 `constant_cmd: [1.0, 0.0, 0.0]`，进入 AMP 后立即按 1 m/s 前进，
此时摇杆和 `/cmd_vel_limiter` 不控制 AMP 速度。真机配置没有此固定指令。**

若要先测零速或用外部速度指令，在终端 A 启动前删除构建产物中的固定指令：

```bash
cd ~/code/F1
sed -i '/^[[:space:]]*constant_cmd:/d' build/cfg/control_module/rl_x1_sim.yaml
```

删除后默认零速。需要行走时在另一个已 source ROS2 的终端持续发布：

```bash
ros2 topic pub -r 10 /cmd_vel_limiter geometry_msgs/msg/Twist \
  '{linear: {x: 1.0}, angular: {z: 0.0}}'
```

停发超过 250 ms 后速度指令归零；下次构建会恢复仓库中的固定指令配置。

## 4. 判断与保存结果

先连续观察至少 12 秒：机器人应保持行走，状态保持 `walk_amp`，没有 `AMP entry rejected`、
`AMP fault` 或切到 `keep`。异常时保存终端输出和：

- `build/log/x1_rl_ctrl.log`
- `build/test_logs/data_csv/walk_diag_*.csv`
- `build/test_logs/data_csv/t_m/` 中的观测日志

可从终端 B 切回站姿：

```bash
ros2 topic pub --once /stand_mode std_msgs/msg/Float32 '{data: 1.0}'
```

终端 A 用 Ctrl+C 停止仿真。若需隔离集成层，先运行
[独立闭环脚本](../scripts/amp_sim_bisect/README.md)；该脚本不需要显示和 ROS2。

本次本地已通过 MuJoCo 3.12.0 的部署模型独立闭环，零速和 1 m/s 均为 600/600 拍（12 秒），
使用 1 ms PD、motor 通道和 ctrlrange 限幅。本机为 macOS，完整 Linux AimRT/ROS2 启动尚未在本机验证。
