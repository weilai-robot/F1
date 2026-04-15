# Sim2Real Checklist

适用对象：`rl_walk_leg.onnx` 在本仓库控制栈上的真机部署与参数适配。

关联资料：
- 部署基线：[deploy_info.json](/Users/yumx/code/X1/agibot_x1_infer/.oma/deploy_info.json:1)
- RL 配置：[rl_x1.yaml](/Users/yumx/code/X1/agibot_x1_infer/src/module/control_module/cfg/rl_x1.yaml:326)
- 控制逻辑：[rl_controller.cc](/Users/yumx/code/X1/agibot_x1_infer/src/module/control_module/src/rl_controller.cc:58)
- 辨识驱动配置：[x1_cfg_identifier.yaml](/Users/yumx/code/X1/agibot_x1_infer/src/install/linux/bin/cfg/x1_cfg_identifier.yaml:1)
- 辨识启动脚本：[run_identifier.sh](/Users/yumx/code/X1/agibot_x1_infer/src/install/linux/bin/run_identifier.sh:1)
- 辨识节点：[native_ros2_ankle_identifier](/Users/yumx/code/X1/agibot_x1_infer/src/assistant/native_ros2_ankle_identifier/main.cc:1)

## 文档结构

- 总体 checklist：本文件，负责维护 sim2real 阶段总览、轮次状态、方案索引、结果索引。
- 具体方案：存放在 `.oma/sim2real/plans/`
- 每轮结果：存放在 `.oma/sim2real/results/`

## 当前状态

| 项目 | 当前值 |
|---|---|
| OMA 阶段 | `deploy` |
| 当前 sim2real 轮次 | `Round 2` |
| 当前轮状态 | `in progress` |
| 当前重点 | 踝关节 `kp/kd` 辨识 |
| 上一轮状态 | `Round 1 completed` |

## 阶段总览

| 阶段 | 状态 | 目标 | 方案 | 最近结果 |
|---|---|---|---|---|
| `sensor_and_sign_check` | completed | 确认传感器、关节顺序、符号、零位无硬错误 | 本阶段按现场基础检查执行 | [Round 1](/Users/yumx/code/X1/agibot_x1_infer/.oma/sim2real/results/round_01_field_test.md:1) |
| `zero -> stand -> hold` | completed | 确认基础 PD 站立稳定 | 本阶段按现场基础检查执行 | [Round 1](/Users/yumx/code/X1/agibot_x1_infer/.oma/sim2real/results/round_01_field_test.md:1) |
| `rl_idle_and_in_place_step` | completed | 确认 RL 零速/小速度下基础行为 | 本阶段按现场基础检查执行 | [Round 1](/Users/yumx/code/X1/agibot_x1_infer/.oma/sim2real/results/round_01_field_test.md:1) |
| `ankle_kp_kd_identification` | in progress | 在改踝关节参数前做闭环辨识 | [ankle_kp_kd_identification.md](/Users/yumx/code/X1/agibot_x1_infer/.oma/sim2real/plans/ankle_kp_kd_identification.md:1) | 待更新 |
| `low_speed_walk` | pending | 在踝关节问题收敛后验证低速直行 | 待创建 | 待更新 |
| `lateral_and_yaw` | pending | 验证横移与转向 | 待创建 | 待更新 |
| `disturbance_and_contact` | pending | 验证扰动和接触鲁棒性 | 待创建 | 待更新 |

## 轮次索引

| 轮次 | 状态 | 目标 | 结果文件 |
|---|---|---|---|
| `Round 1` | completed | 基础链路、站立、RL 小速度初测 | [round_01_field_test.md](/Users/yumx/code/X1/agibot_x1_infer/.oma/sim2real/results/round_01_field_test.md:1) |
| `Round 2` | in progress | 踝关节 `kp/kd` 辨识 | 待生成 |

## 当前结论

- 基础部署链路已跑通。
- `sensor_and_sign_check` 已通过。
- `zero -> stand -> hold` 已通过。
- `rl_idle_and_in_place_step` 已通过基础可用性验证，但仍存在：
  - 行走连续性不足，约 `10 s`
  - 行走形态偏踏步前进
  - 踝关节轻微抖动
- 当前不优先修改：
  - `action_scale`
  - `pd_zero/pd_stand`
- 当前优先动作：
  - 先完成踝关节 `kp/kd` 辨识
  - 再决定优先调整 `kd`、`kp` 还是 `lpf_conf.wc`

## 当前实机辨识启动方式

- 编译：
  - `./build.sh`
- 启动驱动-only：
  - `cd build && ./run_identifier.sh`
- 启动辨识节点：
  - `cd build && ./native_ros2_ankle_identifier --ros-args ...`
- 约束：
  - 辨识时不运行 `run.sh`
  - 辨识时不能有其他节点同时发布 `/joint_cmd`

## 维护规则

- 本文件只维护总览，不写大段实验细节。
- 新的具体实验方案写到 `.oma/sim2real/plans/`
- 每轮真机实验结果单独写到 `.oma/sim2real/results/round_xx_*.md`
- 每完成一轮实验，只更新：
  - 当前 sim2real 轮次
  - 当前轮状态
  - 阶段总览表
  - 轮次索引表
  - 当前结论
