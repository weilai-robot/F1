# AMP 全身运动策略部署适配（x1_29_AMP → F1）

> 分支：`feat/amp-deploy`（F1 主仓库 + motion_control 子模块同名分支）
> 模型：`x1_amp_v46s_soup.onnx`（v46s model soup α=0.5，29DOF 全身 AMP 行走）
> 契约来源：`x1_29_AMP/sim2sim/{deploy_onnx_rt.py, mujoco_rollout.py}`（sim2sim 验证管线）

## 1. 模型验证状态（来自训练侧 v46 验收）

- G2 步态门：walk05 / walk10 / back05 髋膝比全 PASS（0.890~0.985）
- P7 形式门 8/8；walk10 漂移 +0.01 m（历代最优）
- 鲁棒性 51/60（lat2 / lag2 / push0.75 首次全 5/5；push1.0 1/5、worst 0/5 未收敛——**上机避免强推撞**）
- 模型 SHA256：`99f258ebdbbb8fd4d8cc77c956e25b06cb82bd4cbafad7585b82feaaf7348517`

## 2. 部署契约（infer 代码必须逐项满足）

| 项 | 值 |
|---|---|
| 输入 | `obs [1,288] float32`，名称 `obs` |
| 输出 | `action [1,29]`，名称 `action` |
| 归一化 | **融合在 ONNX 图内**（(x−mean)/std 在首层前），C++ 侧喂原始值，不走 obs_scales |
| 每帧布局（96） | `[base_ang_vel(3), projected_gravity(3), cmd(3), joint_pos_rel(29), joint_vel(29), last_action(29)]` |
| 历史展开 | **term-major**：每项 3 帧 oldest-first 拼接（v26 根因修复点，非 frame-major） |
| 历史初始化 | 零初始化（Isaac env reset 行为） |
| 关节序 | LAB_DOF_ORDER = 顶层 `joint_list`（腰3→左臂7→右臂7→左腿6→右腿6） |
| 动作映射 | `q_target = default_q + 0.25 × action` |
| 策略频率 | 50 Hz（decimation=20 @ 1kHz），PD 1kHz |
| 指令范围 | vx∈[-0.5,2.5] vy∈[-0.5,0.5] wz∈[-1.5,1.5]（超出限幅后入 obs） |
| 软启动 | 1.0s（50 拍）零指令零动作，目标角=default_q |
| 重力约定 | `projected_gravity = Rᵀ·[0,0,-1]`，R 为 IMU 四元数旋转阵（body→world） |

## 3. 代码修改点（motion_control 子模块）

| 文件 | 修改 |
|---|---|
| `include/control_module/rl_controller.h` | 新增 `AmpConf`/`amp_mode_`/`amp_settle_remaining_` 成员 |
| `src/rl_controller.cc` | ① Init 解析 `obs_layout: amp` + `amp_conf`（维度校验 9+3n）② RestartController 重置软启动/清 last_actions ③ Update 推理拍软启动零动作 ④ ComputeObservation amp 分支：96 维帧组装、cmd 限幅+软启动置零、零初始化历史、term-major 重排展开 |
| `module/control_module/policy/x1_amp_v46s_soup.onnx` | 模型入库（构建时随 CMake install 到 `build/cfg/control_module/policy/`） |
| `module/control_module/cfg/rl_x1.yaml`、`rl_x1_sim.yaml` | 新增 `rl_walk_amp` 控制器块 + `walk_amp` 状态（trigger `/amp_mode`）+ idle/keep/stand 的 pre_states 增补 |
| `install/linux/bin/cfg/x1_cfg*.yaml`（4 个） | aimrt 通道正则加入 `amp_mode`（允许外部 ROS2 触发） |

**legacy 路径（rl_walk_leg / rl_walk_leg_shoulder）零改动**：`obs_layout` 缺省即 legacy，旧配置行为不变。

## 4. 使用方法

```bash
# 构建
./scripts/build.sh

# 真机（进入 AMP 模式：先站立再切 AMP）
ros2 topic pub --once /zero_mode std_msgs/msg/Float32 "{data: 0}"
ros2 topic pub --once /stand_mode std_msgs/msg/Float32 "{data: 0}"
ros2 topic pub --once /amp_mode std_msgs/msg/Float32 "{data: 0}"
# 速度指令（注意软启动 1s 后才生效；指令超出训练范围会被限幅）
ros2 topic pub -r 10 /cmd_vel_limiter geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
# 退出：/stand_mode 或 /keep_mode（walk_amp → stand/keep/idle 均可达）
```

仿真侧用 `install/linux/bin/cfg/x1_cfg_sim.yaml` 同理。

## 5. 上机前 checklist

1. **关节序核对**：`ros2 topic echo /joint_states` 的 name 顺序无关（按名映射），但确认 29 关节全部在发；`rl_walk_amp.joint_list` 与训练 LAB 序一致（已对拍）。
2. **IMU 四元数**：`/imu/data` orientation 须为 body 在 world 系姿态（ROS 标准）。amp obs 的重力项首次上真机——起步先悬空或吊架验证 obs 波形（walk_diag CSV 有 imu_quat 列）。
3. **软启动**：进 walk_amp 后 1s 内目标角=default_q（LPF 从当前角平滑过渡），勿在软启动期内给指令。
4. **指令范围**：vx>2.5 会被压到 2.5；后退仅验证到 -1.0（OOD），真机限 -0.5。
5. **鲁棒性边界**：push1.0/worst 场景未收敛——避免强推撞、避免突然反向大指令。
6. **力矩限幅**：训练侧 tau_max：腰/髋/膝 180、踝 80、肩/肘 27、腕 10 Nm。dcu_x1.yaml 无显式限幅字段（靠电机自身保护），上机前确认电机层限幅覆盖腕关节（首次由策略驱动腕部）。
7. **诊断数据**：walk_diag CSV 自动记录（20s 上限）；`tm_obs_input_*.bin` 为 288 维原始 obs 序列，可用于离线回放对拍（与训练侧 replay 工具兼容）。

## 6. 与 sim2sim 验证管线的已知差异（有意保留）

| 差异 | 原因 | 风险与对策 |
|---|---|---|
| 位置目标二阶 LPF（wc=100 rad/s ≈ 15.9Hz），踝并联走 effort 模式 | F1 硬件踝并联结构响应不一致的既有缓解 | 若步态发"软"/跟踪差，增大 wc 或去掉非踝关节的 LPF |
| 无 base_height/tilt 看门狗 | 控制模块无基座高度源；与现有 walk_leg 安全策略一致 | 人工急停；后续可从 nav 侧 odom 引入高度 |
| PD 在 dcu 电机层执行（1kHz） | 硬件架构 | 与 Isaac 验证的 ≥200Hz 重算要求满足 |
| 推理在软启动期仍运行（输出被丢弃） | 实现简洁 | 无功能影响 |

## 7. 验证记录（本分支适配时）

- Python 复刻 C++ AMP 路径 vs 训练侧 `ObsBuffer`：40 帧 **bitwise 一致**
- 零初始化语义：首帧 jpos 历史 = [0,0,fr0] ✓
- 真实 ONNX 端到端 200 帧：两路径输出动作 **max diff = 0**，有限有界
- yaml 对拍：LAB 序 / DEFAULT_Q / kp / kd 与训练契约逐位一致；obs=96=9+3×29 ✓
- **C++ 编译未在本机执行**（无 Linux/aimrt 工具链），需 `./scripts/build.sh` 或 CI 验证
