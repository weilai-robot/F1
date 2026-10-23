# X1 v12 / 154 维舞蹈部署

2026-10-23。F1 主仓库与 `motion_control` 子模块分支均为 `feat/x1-dance-v12-154-deploy`。默认舞蹈入口已经切换到 `dance_x1_v12_model_4999.pt` 导出的 154 维 actor；训练仓库为 `~/code/Beyondminic-Weilai-X1`，任务记录为 `TASK_20261022_151` / `Tracking-Flat-X1-Wo-State-Estimation-v0`。

本次完成代码、模型资产、数值仿真与本地独立 C++ 检查。后续已加入[收尾与持续站稳](BeyondMimic_X1_154维收尾与站稳.md)：播放结束改为零速度平衡、落脚门控及监控站立，替代直接进入 keep。**尚未完成 Ubuntu ROS 2 全量构建、控制器 gtest 或真机验收，也没有启动电机。**

## 接口与运行行为

`build/run_dance.sh` → `cfg/x1_cfg_dance.yaml` → `cfg/control_module/rl_x1_dance.yaml`，默认配置：

```yaml
observation_layout: wo_state_estimation
state_source: imu
model_file: cfg/control_module/policy/dance_x1_v12_154_4999.bin
```

模型为 154→512→256→128→29、ELU。保留 checkpoint 的 `_mean`、`_std`，沿用部署归一化 `(obs - _mean) / (_std + 0.01)`。训练仓库 `play_mujoco.py` 则使用 `sqrt(_var + 1e-6)`；因此这里重新使用**部署资产与部署归一化**验证了接管和完整播放。checkpoint 没有保存训练依赖锁或 normalizer eps，本次沿用既有部署的 RSL-RL 默认 eps，未从 checkpoint 独立证明云端 eps。

观测顺序严格去掉两个三维项，未向 160 维 actor 填零，也未把 154 维输入补到 160 维：

| 索引（从 0 开始） | 输入 |
| --- | --- |
| 0–28 | 参考关节位置 |
| 29–57 | 参考关节速度 |
| 58–63 | 当前腰部锚点到对齐参考的相对旋转，矩阵前两列按行展开 |
| 64–66 | IMU 机身坐标角速度 |
| 67–95 | 实测关节位置减训练默认角 |
| 96–124 | 实测关节速度 |
| 125–153 | 上次 action |

姿态来自 IMU 与训练 MJCF 的 base_link→腰三轴→lumbar_pitch_link FK。按 IMU 源时间插值腰部关节，插值区间最多 10 ms，禁止外推；回调先后次序变化时继续使用最新完整且未过期的一对样本。IMU/关节源时间和接收时间仍受 80 ms 时效约束；重复时间戳不能续命。

154 维模式不启动本体位置/速度估计器，不等待接触或加速度偏置校准，不要求关节 effort，也不订阅或等待 `/dance/odometry`、`/dance/anchor_pose`、`/dance/base_velocity`。它仍要求有效的 IMU 姿态、角速度、29 关节位置/速度与腰部时间配对。

按键保持 **A → B → Guide**：零位 → stand → 4 秒过渡到训练 default_q → 连续就绪 4 秒 → 自动从帧 0 播放。154 维准备阶段检查关节误差 ≤0.12 rad、关节速度 ≤0.5 rad/s、IMU 角速度模 ≤0.2 rad/s、倾角 ≤35°、启动关节限位与新鲜反馈；不再以不可用的机身线速度作为等待条件。角速度与关节速度门限仅用于准备就绪，不是播放退出条件。

播放保持 1 kHz 控制、每 20 拍推理一次的 50 Hz / ZOH。动作缩放、default_q、kp/kd、effort、armature 和参考动作与 v11 相同。普通关节下发位置和 kp/kd，踝关节每 1 ms 用最新反馈计算软件 PD 并限幅 ±80 Nm。保留目标限位、35° 倾角、反馈超时、非有限观测/动作、过大 action 保护和 Back/Start 退出；未重新加入已移除的位置偏差或关节跟踪误差退出条件。647 帧播放结束后进入专用零速度平衡收尾，连续验证站稳后保持 `dance_stand`。

## 已执行验证

- 序列化模型与独立 PyTorch actor 对照 129 组输入，最大绝对动作误差 `1.91e-5`；错误观测布局在导出前被拒绝。
- v11 重新导出得到与原资产完全相同的模型/参考 SHA256；旧 160 维模式的 yaw=0 准备及播放探针也完成 647 帧。
- Clang + AddressSanitizer / UndefinedBehaviorSanitizer：生产腰部 FK 的 70 组独立 MuJoCo 对照、时间插值/环形历史检查通过；诊断日志的 154/160 列数、边界、精度、异步 flush/队列溢出检查通过。
- 使用生产 C++ 腰部 FK、部署 YAML/资产的 Python/MuJoCo 探针，从已站立状态驻留 3 秒，执行准备及自动接管；三组均满足连续就绪并完成整段播放。

| 初始 yaw | actor 帧数 | 最大倾角 | 最低 base z | 对齐参考的平均真实锚点位置误差 |
| --- | --- | --- | --- | --- |
| 0 | 647 | 29.75° | 0.508 m | 0.827 m |
| 0.8 | 647 | 29.74° | 0.507 m | 0.899 m |
| −1.2 | 647 | 29.75° | 0.508 m | 0.894 m |

原始结果和来源校验见 [154 维验证记录](analysis/x1_dance_v12_154_20261023/README.md)。探针未运行完整 ROS 控制器、网络/DCU 或实际执行器，也未模拟冷启动 A/B 阶段。位置误差用仿真真值独立计量，没有反馈到 actor。154 维模型失去了直接的位置误差与线速度输入，整段完成并不证明跟随参考的全局路线；现场应单独评估实际位移。这批早期播放记录也不覆盖动作结束后的稳定性；收尾验收以[新增停止验证](analysis/x1_dance_v12_154_finish_20261023/README.md)为准。

新增控制器 gtest 覆盖模型/布局/状态源组合拒绝、无 odometry/effort/加速度校准也可接管、154 维逐项独立 golden 输入及输出、静止门控、重复/过期反馈、源时间配对、20 拍目标保持与逐拍踝 PD。这些 ROS/Eigen/gtest 用例尚未在本机运行。

诊断 CSV 升级到 `schema_version=2`，包含 `observation_dim=154`，只输出 `obs_raw_0…153` 和 `obs_normalized_0…153`。不可用的身体平移、线速度、锚点位置/误差保留 NaN，相关身体/速度时间戳为 0，`estimator_valid=0`；这在 154 维模式属于正常状态。读取日志时应按列名及观测维数解析。

## Ubuntu 构建与启动

F1 与 motion_control 的同名部署分支均已推送到 origin；主仓库固定引用对应子模块提交。保留服务器本地修改后再更新：

```bash
cd ~/code/F1
git fetch origin
git switch feat/x1-dance-v12-154-deploy
git pull --ff-only
git submodule update --init --recursive motion_control
./scripts/build.sh -DXYBER_X1_INFER_BUILD_TESTS=ON
ctest --test-dir build --output-on-failure -R '(control_module|joy_stick_module)'
```

测试通过后，使用 Release 配置关闭测试选项并重新构建，再启动舞蹈入口：

```bash
./scripts/build.sh -DXYBER_X1_INFER_BUILD_TESTS=OFF
cd build
bash ./run_dance.sh
```

先等每一步到位提示，再按 A、B、Guide。Guide 已授权准备后自动播放，重复 Guide 不会重置准备；结束后自动执行收尾，确认站稳进入 `dance_stand`，下一次从 A 开始。诊断日志位于 `build/test_logs/data_csv/dance_diag_*.csv`，旁边的 `.csv.yaml` 保存实际配置。默认模式无需启动额外身体估计节点；收尾的载荷判断和静态站姿确认需要有效双腿力矩反馈，缺失时保持零速度平衡等待。

v11 模型与完整状态路径仍保留，旧 YAML 另存为 `cfg/control_module/rl_x1_dance_v11.yaml`；它需要原先的完整状态来源。模型维数、`observation_layout` 和 `state_source` 组合不匹配时初始化会拒绝。

## 离线复现

在具备 NumPy、PyTorch、MuJoCo、SciPy、PyYAML 和 Clang 的环境，从 F1 根目录运行：

```bash
python motion_control/module/control_module/tools/export_beyondmimic_x1.py \
  --training-root ~/code/Beyondminic-Weilai-X1 \
  --checkpoint ~/code/Beyondminic-Weilai-X1/trained_models/dance_x1_v12_model_4999.pt \
  --motion ~/code/Beyondminic-Weilai-X1/whole_body_tracking/motions/dance_x1_train.npz \
  --observation-layout wo_state_estimation \
  --model-out motion_control/module/control_module/policy/dance_x1_v12_154_4999.bin \
  --motion-out motion_control/module/control_module/policy/dance_x1_motion.bin
python motion_control/module/control_module/tools/check_beyondmimic_x1_154.py \
  --training-root ~/code/Beyondminic-Weilai-X1
python motion_control/module/control_module/tools/sim_beyondmimic_x1.py \
  --training-root ~/code/Beyondminic-Weilai-X1 --yaw 0
```

重新导出旧 160 维模型时显式使用 `--observation-layout full`；仿真时通过 `--config motion_control/module/control_module/cfg/rl_x1_dance_v11.yaml` 选择旧配置。
