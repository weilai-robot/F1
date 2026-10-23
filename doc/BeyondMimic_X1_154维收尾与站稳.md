# X1 154 维舞蹈：收尾、落脚与持续站稳

本版修复动作播放结束后立即进入 keep 导致倒地的问题。647 帧舞蹈及其权重保持不变，播放名义时长仍为 12.94 秒；后面新增受反馈门控的收尾阶段。当前已验证生产 C++ 收尾核心与部署模型的离线闭环，完整 Ubuntu ROS 2/DCU 构建和真机结果仍待验收。

## 当前流程

`A → B → Guide → 准备 → 12.94 秒舞蹈 → dance_finish → dance_stand`

| 阶段 | 控制行为 | 进入下一阶段的条件 |
| --- | --- | --- |
| finish_brake | 从舞蹈最后实际下发的目标，用 0.1 秒五次曲线混合到零速度 AMP 平衡策略 | 0.1 秒混合完成 |
| finish_land | AMP 保持 `[vx,vy,wz]=[0,0,0]`，继续调整支撑、减小运动；1 秒后开始用 2 秒把上身目标收向名义姿态 | 上身收拢完成，且连续 0.75 秒满足双脚载荷/几何一致与低速度条件 |
| finish_stand_blend | 用 4 秒五次曲线把零速度 AMP 目标及增益渐变到 B 键 `pd_stand` 的姿态和增益；过渡期间逐步减少 AMP 平衡修正 | 混合完成；异常则回到平衡恢复 |
| finish_standing | 保持 B 键对应的 stand 位，同时持续监控支撑、速度、姿态和关节到位误差 | 全部关节到位误差 ≤0.12 rad，且再连续安静 2 秒后，状态机显示 `dance_stand`；控制仍持续运行 |

最终目标与 B 键 `pd_stand` 相同，初始化时按关节名称校验姿态及 kp/kd，避免两种关节顺序造成错配。先由已有 AMP 策略在零速度命令下完成落脚和平衡，再渐变回 stand 位。`dance_stand` 保留收尾控制器的支撑监控及恢复能力。

三朝向离线测试的实际确认时间见[回 stand 验证](analysis/x1_dance_v12_154_return_stand_20261023/README.md)。条件不足会继续平衡等待，经过固定时长不会自动宣告成功。站稳后不自动切 keep，也不关闭姿态监控。

## 模型和接口

- 舞蹈继续使用 `dance_x1_v12_154_4999.bin`，154 维单帧输入。
- 收尾专用 `rl_dance_finish` 使用仓库已有的 `x1_amp_v70b_m16000_50hz.onnx`，288 维历史输入；其 normalizer 已在 ONNX 图内。两套模型各用自己的观测和动作契约。
- 收尾配置显式固定零速度指令，忽略外部行走命令。初始化拒绝非零固定速度、settle 静持、额外 LPF 或与舞蹈不一致的关节顺序/名义 PD/default_q。
- 1 kHz 控制、50 Hz 策略、目标限位保持。站立阶段的踝关节仍由主机逐拍计算软件 PD、限幅 ±80 Nm，不切换为未经验证的踝电机位置环。
- 腰/臂/腕目标另外限制为 2 rad/s 的目标变化率；这不等同于实测电机速度保证。

舞蹈 actor 不新增位置或线速度输入。收尾通过 IMU、双腿 q/qd/力矩与训练腿部几何，计算准静态载荷线索和双支撑下的身体速度线索；不要求 odometry 或加速度偏置校准。这些是接触/运动估计，不是脚底实测力传感器，也不能独立检测持续整体滑移。

静态站姿接管前要求：

- 两脚载荷通过 35/15 N 滞回与 0.1 秒滤波；最低脚端高度差 ≤3 cm，脚端相对速度差 ≤0.15 m/s。
- 倾角 ≤10°，IMU 角速度模 ≤0.1 rad/s，全部关节速度 ≤0.3 rad/s，接触运动学速度模 ≤0.025 m/s。
- 源时间递增，关节/IMU 源时间差 ≤10 ms。重复时间戳不积累就绪时间。

腿部力矩缺失/非有限或时间不能配对时，仍可用 q/qd/IMU 运行零速度平衡，但不会宣称站稳或强行交权给静态站姿。关节/IMU 必需反馈超过 80 ms、倾角超过 35°、模型或观测非有限则走既有故障退出路径。

站立中，倾角 >12°、角速度 >0.3 rad/s、估计速度 >0.12 m/s 或支撑不一致持续 50 ms，会重新进入零速度平衡恢复，之后再次门控和验证。恢复是尝试，不保证任意外力或地面条件下都能成功。

## 验证范围

[当前回 stand 数值结果、来源哈希和失败候选](analysis/x1_dance_v12_154_return_stand_20261023/README.md)；[此前捕获稳定姿态的历史验证](analysis/x1_dance_v12_154_finish_20261023/README.md)单独保留。仿真调用生产 `DanceFinishCore`。MuJoCo 真值的世界速度、接触和位移仅用于独立评分，不进入收尾核心或 AMP actor。

通过条件为：完整舞蹈完成，收尾确认站稳后连续至少 10 秒满足真值倾角 <10°、线速度 <0.05 m/s、角速度 <0.2 rad/s、全部关节速度 <0.5 rad/s、双脚触地、相对 B 键 stand 的最大关节误差 ≤0.12 rad，并且该 10 秒水平位移 ≤5 cm。加入三个明确幅度的合成噪声种子及站稳后持续 0.15 秒的前后向 30/60 N 推力试验。

独立 Clang + ASan/UBSan 检查覆盖：由 MuJoCo 生成的承重/悬空夹具、状态和连续就绪门控、固定 stand 目标及未到位不得确认、恢复、重复时间戳、限位/目标限速、时钟停顿及倾角保护。ROS/Eigen/ONNX 控制器 gtest 已补入，但本地 Mac 未运行完整套件。

仿真使用理想的 effort 限幅关节 PD，尚未覆盖真实并联踝传动、L28 腕部位置环、真实时延/噪声、地面滑移或不同机械标定。离线通过不能当作真机已验收。

## 远端更新与验收

分支仍为 `feat/x1-dance-v12-154-deploy`。保存远端修改后，从 F1 根目录更新主仓库与子模块，按[部署说明](BeyondMimic_X1_154维舞蹈部署.md)构建和启动：

```bash
git pull --ff-only
git submodule update --init --recursive motion_control
source scripts/ros2_source.sh
./scripts/build.sh Release -DXYBER_X1_INFER_BUILD_TESTS=ON
ctest --test-dir build --output-on-failure -R '(control_module|joy_stick_module)'
# 上面测试通过后再构建运行版本
./scripts/build.sh Release -DXYBER_X1_INFER_BUILD_TESTS=OFF
cd build
bash ./run_dance.sh
```

现场首先验证完整播放后的自动收尾和至少 10 秒稳定保持。日志应依次出现 `finish_brake`、`finish_land`、`finish_stand_blend`、`finish_standing`，最后出现 `Dance stopping complete`。等待中会给出载荷、速度或时间配对原因；没有该完成提示，不能记作站稳通过。

`dance_diag_*.csv` 中 `observation_dim=0`、`state_source=foot_kinematics` 的文件为收尾诊断，包含阶段、q/qd、载荷线索、接触速度和实际命令；旁边 `.yaml` 保存配置。AMP 原生 `walk_diag` 与 `tm_obs_input` 记录原始动作/模型输入。0 维只表示这份收尾诊断不重复保存 actor 观测，不代表 AMP 输入为 0 维。

Back/Start 仍可随时退出到 keep/idle；它们属于人工退出，不是经过验证的自动站稳流程。再次跳舞从 A、B、Guide 开始。

## 离线复现

在具备 NumPy、PyTorch、MuJoCo、SciPy、PyYAML、ONNX Runtime、C++ 编译器的环境运行：

```bash
python motion_control/module/control_module/tools/sim_dance_finish_x1.py \
  --training-root ~/code/Beyondminic-Weilai-X1 --yaw 0 --out /tmp/finish_yaw0.json
python motion_control/module/control_module/tools/sim_dance_finish_x1.py \
  --training-root ~/code/Beyondminic-Weilai-X1 --noise-seed 23 --out /tmp/finish_noise23.json
python motion_control/module/control_module/tools/sim_dance_finish_x1.py \
  --training-root ~/code/Beyondminic-Weilai-X1 --push 60 --out /tmp/finish_push60.json
```
