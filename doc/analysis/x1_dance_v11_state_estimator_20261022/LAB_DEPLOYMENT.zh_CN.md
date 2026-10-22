# X1 v11 实验室部署步骤

本体估计代码位于 `motion_control` 子模块的 `feat/x1-dance-v11-deploy` 分支，F1 的 `feat/x1-beyondminic-dance-deploy` 分支固定对应子模块提交。在线部署应更新主仓库和子模块；只更新 F1 而不检出子模块会漏掉控制器代码。

## 1. 同步和构建

使用单独的舞蹈工作副本，保留已跑通的 AMP 版本。先保存当地未提交修改和实机参数，再在 F1 根目录更新：

```bash
cd /path/to/F1
git fetch origin
git switch feat/x1-beyondminic-dance-deploy
git pull --ff-only
git submodule update --init --recursive motion_control
git -C motion_control status --short
# 正常应没有运动控制源码修改；子模块处于 detached HEAD 属正常情况。

bash scripts/build.sh -DXYBER_X1_INFER_BUILD_TESTS=ON
ctest --test-dir build --output-on-failure -R '(control_module|joy_stick_module)'
# 确认测试通过，再关闭 coverage 构建运行版本：
bash scripts/build.sh Release -DXYBER_X1_INFER_BUILD_TESTS=OFF
```

此前的 `x1_dance_v11_lab_20261022.tar.gz` 是提交前生成的离线覆盖包，基线为 F1 `f6bc0200324353f165c21a1be36e04bbd82d2097`、motion_control `5d66c33737f3ba084cdff22d12cf4531d90cdac6`；`lab_sync_manifest.json` 仅记录该历史包的文件 hash。在线更新后无需再覆盖此旧包。

若测试编译或 ctest 失败，保留输出并先处理失败，不进入电机测试。当前已运行 Mac 上的 C++ 估计核心检查，完整 ROS/Eigen/gtest 编译是本次实验室需要补做的检查。

## 2. 核对实际运行配置

构建会重新复制配置，以下核对在最后一次构建后进行，现场修改 `build/cfg/`：

- `build/cfg/dcu_driver_module/dcu_x1.yaml`：沿用该机器已通过 AMP 的网卡、DCU 顺序、传动/方向配置。源码默认网卡 `enp2s0`、EtherCAT CPU 4，必须与机器一致。
- `build/cfg/control_module/rl_x1_dance.yaml`：`use_sim_handles: false`，`initial_state: idle`，`state_source: proprioceptive`；控制 CPU 默认 6，机器应存在并可绑定该 CPU。
- 舞蹈配置的 `joint_offset` 应与该机器人已验证的模型坐标校准一致；代码更新会更新舞蹈 YAML，不能忽略当地校准值。不要二次反转腰 pitch。
- `controllers.dance_x1.state_estimator`：下肢 DCU 的比力单位必须是 m/s²，静止模长约 9.81。确认其轴向与 base 一致；`imu_position_in_base` 当前为训练默认零，应填写已核实的实机安装偏置（base 坐标，米），不凭猜测修改。

本轮不需要启动 FastLIO2、Nav2 或 `dance_state_bridge.py`。同一套电机上只运行一个控制进程。

## 3. 首次仅检查反馈

将 `build/cfg/dcu_driver_module/dcu_x1.yaml` 中 `enable_actuator` 改成 `false`。机器人由吊架/支撑承托，使用直接启动方式，不调用会使能颈部舵机的启动脚本：

```bash
cd /path/to/F1/build
source ./ros2_source.sh
source ./ros2_setup.sh
sudo setcap cap_net_raw,cap_sys_nice=ep ./aimrt_main
./aimrt_main --cfg_file_path=./cfg/x1_cfg_dance.yaml
```

另一个终端进入同一 `build` 并 source 同样两个文件；所有终端保持相同 ROS_DOMAIN_ID/RMW 设置：

```bash
timeout 6s ros2 topic echo /imu/data --once
timeout 6s ros2 topic echo /joint_states --once
timeout 6s ros2 topic hz /imu/data
timeout 6s ros2 topic hz /joint_states
```

检查 IMU 四元数、静止比力和关节位置/速度/力矩是否合理且持续更新。双腿需要有效力矩反馈。若比力约为 1 或约为 0，先查清单位/是否已去重力，不能直接开始播放。电机未使能或完全悬空时，没有承重状态，`/dance/odometry` 不发布属于可能的正常现象。

完成后退出检查进程。硬件参数修改如果要长期保存，也应回填对应源码配置；再次构建会覆盖 `build/cfg/`。

## 4. 使能后先验证 stand 和估计

在退出进程后，把 `enable_actuator` 恢复 `true`。吊架防倒，双脚能接地实际承重。按现场既有 AMP 使能流程确认机器人和颈部保持状态，然后启动舞蹈入口：

```bash
cd /path/to/F1/build
bash ./run_dance.sh
```

另一终端在 F1 根目录依次执行，或使用手柄 A、B；每一步等到对应日志再执行下一步：

```bash
bash scripts/switch_x1_mode.sh zero
# 等待：Zero reached; press B for stand
bash scripts/switch_x1_mode.sh stand
# 等待：Stand reached; press Guide to prepare and dance
```

保持在 stand，尚不按 Guide。检查终端在 build 目录 source ROS 环境后执行：

```bash
timeout 6s ros2 topic echo /dance/odometry --once
timeout 6s ros2 topic hz /dance/odometry
tail -n 60 log/x1_rl_ctrl_dance.log
```

期望 `/dance/odometry` 约 50 Hz，帧为 `odom -> base_link`；静止速度接近零、身体高度合理，无明显跳变/持续漂移。完成校准需要至少 0.5 秒连续静止承重；完全吊离地或卸载过多不能满足此条件。

若没有估计输出，先检查脚承重、比力单位、力矩反馈和源时间；不要切换为仿真时间、放宽超时或补零绕过检查。当前接触估计和安装参数没有在这台实机验收。

## 5. 记录并执行一次自动播放

保持同一 ROS 环境，可在独立终端记录（`/tmp` 输出目录应尚不存在）：

```bash
ros2 bag record -o /tmp/x1_dance_v11_first \
  /imu/data /joint_states /dance/odometry /joint_cmd
```

站立和估计检查通过后，只按一次 Guide，或在 F1 根目录执行：

```bash
bash scripts/switch_x1_mode.sh dance-start
```

Guide 会授权整段“4 秒准备位过渡 → 连续就绪 4 秒 → 自动播放”，无需第二次按键，也不会停在准备位等确认。任一就绪条件不满足，等待会延长并每 2 秒输出原因。

正确日志顺序：

1. `Guide accepted`。
2. `Ready hold complete; starting dance automatically from frame 0`。
3. 播放约 12.94 秒后 `Dance motion completed; switching to keep mode`。

下沉到准备位不能代表 policy 已运行。若出现 `Dance preparation:`，读其等待原因；若出现 `Dance fault:`，保留完整故障日志并按具体原因排查。

现场反馈后的版本已移除播放阶段的 `dance joint tracking error or limit exceeded` 和 `dance anchor tracking error exceeded limit` 看门狗，实测关节越限、瞬时目标跟踪误差或锚点位置偏差不再自动切 keep。锚点偏差仍输入 policy，参考只在播放开始时对齐一次。启动前的关节限位检查、输出目标裁剪、踝力矩限幅、倾角/反馈超时退出及 Back/Start 遥控退出仍保留。此改动需要更新 `motion_control` 并重新编译，单改 YAML 或重启旧程序不会移除旧判断；旧 YAML 的 `max_tracking_error`、`max_anchor_error` 字段在新程序中不再生效。

## 6. 退出与带回结果

- Back / `bash scripts/switch_x1_mode.sh keep`：中止流程，保持当前实测姿态。它是软件保持，不等于硬件急停。
- Start / `bash scripts/switch_x1_mode.sh idle`：进入 idle，位置刚度、阻尼和前馈力矩均为零。先用吊架承托，再执行；现场硬件急停按设备流程使用。
- 正常结束后先确认 keep，用吊架承托，再 idle、退出控制程序，并按原实验室流程下电。

带回编译/ctest 输出、`build/log/x1_rl_ctrl_dance.log*`、本次 bag 和实际配置文件。首次实机评价重点是估计输出是否连续、静止漂移、准备/接管是否成功，以及每次保护的具体原因；离线三组通过不能替代这些检查。
