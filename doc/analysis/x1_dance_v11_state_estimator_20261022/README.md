# X1 v11 本体状态估计验证（2026-10-22）

已补齐 v11 的身体位置/线速度来源。默认 `state_source: proprioceptive`，输入只有机身 IMU 与双腿编码器/关节力矩；身体状态经源时间腰部 FK 进入既有 160 维 actor。检查话题 `/dance/odometry` 发布 50 Hz，内部估计随传感器运行；保留外部 odometry 模式。未改策略权重、动作参考、PD 增益、4 秒就绪等待或保护阈值。

## 实现与适用范围

- IMU 姿态提供完整 roll/pitch/yaw，SI 比力进行惯性预测；0.5 秒静止承重窗口校准加速度偏置。身体输出速度转换到 base_link，源时间不改写。
- 双腿 FK、Jacobian、重力项由训练模型几何和质量得到。关节力矩的准静态载荷估计仅作接触线索，叠加足端速度一致性检查，避免把摆腿惯性当成落地；这不是官方 G1 的完整广义动量观测器。
- 每轴滤波状态是身体位置/速度/加速度偏置及两脚接触点；接触足端位置、零速度和平地高度用于校正。选择脚底最低球面点，接触点切换时保留基座协方差。
- 仅在关节源时间间隔 ≤10 ms 时插值，不外推。源时间缺口 >10 ms 重校准；可信接触丢失 >0.35 秒使估计失效。既有 80 ms 时效、35° 倾角、0.6 m 锚点和关节限位保护未放宽。
- 需要已对齐 base 轴向的 IMU 和有效关节力矩，静止比力模长约 9.81。IMU 平移偏置默认训练值零，支持角加速度/向心项补偿。实机需确认下肢 DCU 的单位、安装偏置和力矩反馈；完全悬空不完成校准。本体估计无法独立观测持续滑移和 yaw 漂移。

## 估计反馈闭环结果

Mac / MuJoCo 3.14.0，训练 MJCF、v11 原资产，1 kHz 物理/PD、50 Hz actor；stand 驻留 3 秒、高增益过渡 4 秒、连续就绪 4 秒。所有状态反馈在同一仿真源时间读取，`mj_forward` 消除了 `mj_step` 后传感器字段仍处于积分前状态的问题。旧报告不与本次逐位比较。

**实际生产 C++ 估计核心通过 ctypes 参与闭环。** 估计器不读取 MuJoCo 根位姿/速度或接触真值；仿真真值只用于评分、既有 IMU/编码器传感器模拟和物理执行。策略的身体位置和线速度来自估计值，锚点由估计基座状态和腰部关节计算。Python 执行 actor/PD/状态流程，没有运行完整 Ubuntu C++ 控制器、ROS 2 或 DCU。

位置误差只移除首次就绪时不可观测的水平原点，保留高度误差。以下 RMSE 只统计播放阶段，非准备阶段稀释后的全程 RMSE。锚点真实误差是机器人相对已对齐参考轨迹的误差，不是状态估计误差。

| 初始 yaw (rad) | 真值/估计完成帧 | 位置 RMSE (cm) | 速度 RMSE (m/s) | 平均真实锚点误差：真值/估计 (m) | 最大估计锚点误差 (m) |
| --- | --- | --- | --- | --- | --- |
| 0 | 647 / 647 | 1.78 | 0.0130 | 0.1333 / 0.1231 | 0.3510 |
| 0.8 | 647 / 647 | 1.93 | 0.0130 | 0.1437 / 0.1243 | 0.3502 |
| -1.2 | 647 / 647 | 2.06 | 0.0129 | 0.1416 / 0.1896 | 0.5228 |

三组均无估计失效或播放保护退出；这仅覆盖这些初始状态。yaw=-1.2 的跟踪误差增加，不能据此宣称估计反馈等价于真值反馈。

额外合成噪声探针 seed=22 完成 647 帧：只扰动估计器输入，标准差为 gyro 0.002 rad/s、accel 0.05 m/s²、关节位置 0.001 rad、速度 0.01 rad/s、力矩 0.5 Nm，另加固定 accel 偏置 [0.04,-0.03,0.08] m/s²。该探针没有替换 actor 的其余本体感知输入，也没有模拟 IMU 姿态误差、网络延迟或硬件噪声统计。

## 已执行检查

- 实际 C++ 核心：Clang C++17 + AddressSanitizer / UndefinedBehaviorSanitizer，覆盖静止偏置/高度、承重校准、错误 SI 单位、无载悬空、冻结/无效输入、断流重校准、短时惯性传播、body 速度坐标系和持续失去支撑。
- 同源输入同步：IMU/关节到达顺序、插值、重复源时间、缺少右侧样本和超 10 ms 间隔拒绝。
- 独立 MuJoCo 对照：100 组随机机身/关节状态，逐项核对足端 FK、点 Jacobian、速度和重力/载荷求解；已有 70 组腰部 FK/时间插值检查继续通过。
- 模型生成器重跑与落盘头文件一致；YAML、Python 语法、C++ warning 检查和 diff whitespace 检查。
- 新增 ROS 控制器回归：内置估计无外部 odometry 的就绪、源时间输出、乱序回调、冻结输入、参数拒绝；**这组 ROS/Eigen/gtest 测试尚未在 Ubuntu 编译运行**。

初始候选和失败记录保存在 `initial_candidates/`。最初接触误判造成漂移，闭环先后触发关节限位、锚点保护或估计失效。最终通过调整接触一致性与观测噪声权重解决这些有限场景，未改退出保护。

## 复现

Python 环境需 numpy、torch、mujoco、scipy、PyYAML；C++ 编译器支持 C++17。无需运行 ROS 或电机。

```bash
python motion_control/module/control_module/tools/check_dance_estimator.py \
  --training-root /path/to/Beyondminic-Weilai-X1
python motion_control/module/control_module/tools/sim_beyondmimic_x1.py \
  --training-root /path/to/Beyondminic-Weilai-X1 \
  --state-feedback proprioceptive --yaw 0 \
  --json-out /tmp/estimated_yaw0.json
# 对照使用 --state-feedback truth；另外两朝向 --yaw 0.8 / --yaw -1.2。
# 噪声探针增加 --estimator-noise-seed 22。
```

源码哈希见 `source_hashes.json`，每份 JSON 记录完整估计参数和核心 hash。模型和参考的来源 hash 继续保存在原 policy JSON。Ubuntu 下一步执行部署说明中的完整构建/ctest，随后检查默认内置估计的 `/dance/odometry`，再验证实机传感器单位、安装参数与吊架落地承重运行；这些尚未执行。
