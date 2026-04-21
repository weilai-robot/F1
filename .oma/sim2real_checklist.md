# Sim2Real Checklist

适用对象：`rl_walk_leg.onnx` 在本仓库控制栈上的真机部署与参数适配。

关联资料：
- 部署基线：[deploy_info.json](/Users/yumx/code/X1/agibot_x1_infer/.oma/deploy_info.json:1)
- RL 配置：[rl_x1.yaml](/Users/yumx/code/X1/agibot_x1_infer/src/module/control_module/cfg/rl_x1.yaml:326)
- 控制逻辑：[rl_controller.cc](/Users/yumx/code/X1/agibot_x1_infer/src/module/control_module/src/rl_controller.cc:58)
- 辨识驱动配置：[x1_cfg_identifier.yaml](/Users/yumx/code/X1/agibot_x1_infer/src/install/linux/bin/cfg/x1_cfg_identifier.yaml:1)
- 辨识启动脚本：[run_identifier.sh](/Users/yumx/code/X1/agibot_x1_infer/src/install/linux/bin/run_identifier.sh:1)
- 辨识模块配置：[ankle_identifier.yaml](/Users/yumx/code/X1/agibot_x1_infer/src/module/ankle_identifier_module/cfg/ankle_identifier.yaml:1)
- 辨识模块实现：[ankle_identifier_module.cc](/Users/yumx/code/X1/agibot_x1_infer/src/module/ankle_identifier_module/src/ankle_identifier_module.cc:1)

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
| 当前重点 | 补悬空工况，并用 `tracking_ratio + damping quality` 重新收口踝关节辨识 |
| 上一轮状态 | `Round 1 completed` |

## 阶段总览

| 阶段 | 状态 | 目标 | 方案 | 最近结果 |
|---|---|---|---|---|
| `sensor_and_sign_check` | completed | 确认传感器、关节顺序、符号、零位无硬错误 | 本阶段按现场基础检查执行 | [Round 1](/Users/yumx/code/X1/agibot_x1_infer/.oma/sim2real/results/round_01_field_test.md:1) |
| `zero -> stand -> hold` | completed | 确认基础 PD 站立稳定 | 本阶段按现场基础检查执行 | [Round 1](/Users/yumx/code/X1/agibot_x1_infer/.oma/sim2real/results/round_01_field_test.md:1) |
| `rl_idle_and_in_place_step` | completed | 确认 RL 零速/小速度下基础行为 | 本阶段按现场基础检查执行 | [Round 1](/Users/yumx/code/X1/agibot_x1_infer/.oma/sim2real/results/round_01_field_test.md:1) |
| `ankle_kp_kd_identification` | in progress | 在改踝关节参数前做闭环辨识 | [ankle_kp_kd_identification.md](/Users/yumx/code/X1/agibot_x1_infer/.oma/sim2real/plans/ankle_kp_kd_identification.md:1) | 触地首轮完成，但悬空工况缺失，且评价准则已修正 |
| `low_speed_walk` | pending | 在候选踝关节参数下验证低速直行、连续性和抖动变化 | [round_03_low_speed_walk_with_ankle_candidates.md](/Users/yumx/code/X1/agibot_x1_infer/.oma/sim2real/plans/round_03_low_speed_walk_with_ankle_candidates.md:1) | 已有草案，但必须等待 Round 2 关闭后才能执行 |
| `lateral_and_yaw` | pending | 验证横移与转向 | 待创建 | 待更新 |
| `disturbance_and_contact` | pending | 验证扰动和接触鲁棒性 | 待创建 | 待更新 |

## 轮次索引

| 轮次 | 状态 | 目标 | 结果文件 |
|---|---|---|---|
| `Round 1` | completed | 基础链路、站立、RL 小速度初测 | [round_01_field_test.md](/Users/yumx/code/X1/agibot_x1_infer/.oma/sim2real/results/round_01_field_test.md:1) |
| `Round 2` | in progress | 踝关节 `kp/kd` 辨识补测与判据修正 | [round_02_ankle_kp_kd_identification.md](/Users/yumx/code/X1/agibot_x1_infer/.oma/sim2real/results/round_02_ankle_kp_kd_identification.md:1) |
| `Round 3` | pending | 分轴踝关节参数下的低速步态验证 | 待生成 |

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
- `Round 2` 当前进展：
  - 辨识链路已切换为 `run_identifier.sh -> DcuDriverModule + AnkleIdentifierModule`
  - 不再依赖外部 `native_ros2_ankle_identifier` ROS2 topic bridge
  - 四个自由度都已完成“完全着地”工况首轮阶跃辨识
  - `right roll` 已补悬空 `kp=35, kd=0.5/0.8/1.0`
  - `right pitch` 已补悬空 `kp=35, kd=0.5`，并补完完全触地 `kd=0.5` 支路
  - `left pitch` 与 `left roll` 仍缺悬空数据，`right pitch 100/0.8` 也仍缺悬空对照，因此 `Round 2` 不能关闭
  - 数据分析判据已从“无超调优先”修正为“`tracking_ratio` 接近 `1.0` 且无振荡优先”
- 当前判断：
  - 仅凭 `no_overshoot + no_zero_crossing` 不能证明参数好
  - 触地首轮里多个配置存在明显欠跟踪，属于“系统偏软”而非“阻尼优良”
  - `right pitch 100/0.8` 与 `right roll 60/0.8` 目前只是触地下的相对较优点，不是最终收敛结论
  - `right roll` 触地下 `35/0.5` 可作为相对最好对照点，但不能当成最终候选
  - `right roll` 悬空工况下，`kp=35` 配合 `kd=0.5/0.8/1.0` 都不是可收口点：
    - `0.5` 为持续振荡
    - `0.8/1.0` 改善为单次过冲，但仍是过冲后回落
  - 这说明 `right roll` 对接触条件高度敏感，当前更应关注悬空/触地等效动力学差异，而不是把 `kp/kd` 看成单调可调
  - `right pitch` 悬空工况下，`kp=35, kd=0.5` 已接近可用区间：
    - `peak/tail tracking` 接近 `1`
    - 时间响应满足 walking 预算
    - 但仍有轻度振荡和轮次分裂，属于接近可用但未收口
  - `right pitch` 完全触地 `kd=0.5` 支路已可判定为无效方向，不应继续沿这条线加 `kp`
  - `left pitch` 与 `left roll` 仍需要继续扫描或复核工况一致性
- `Round 3` 当前策略：
  - 已形成草案，但继续暂缓
  - 待 `Round 2` 在悬空和触地两类工况下都形成闭环结论后，再决定是否进入低速步态验证

## 当前实机辨识启动方式

- 工作流约束：
  - 本地电脑只负责修改源码、配置、分析脚本和部署文档。
  - 真正的编译与 `run_identifier.sh` 在实验室电脑执行。
  - 推荐流程：本地改完后 `git push`，实验室电脑 `git pull` 后再编译和测试。

- 编译：
  - 在实验室电脑执行 `./build.sh`
- 启动辨识进程：
  - 在实验室电脑执行 `cd build && ./run_identifier.sh`
- 参数入口：
  - 本地修改 [ankle_identifier.yaml](/Users/yumx/code/X1/agibot_x1_infer/src/module/ankle_identifier_module/cfg/ankle_identifier.yaml:1) 中的 `test_side`、`test_axis`、`test_kp`、`test_kd`、`step_amplitude_rad`、`csv_path`
  - 或使用 [set_ankle_identifier_config.py](/Users/yumx/code/X1/agibot_x1_infer/.oma/sim2real/set_ankle_identifier_config.py:1) 一次性切换源码配置与已生成运行配置
- 结果分析：
  - 实验室电脑产出 CSV 后，可在实验室电脑或同步回本地后执行 `python3 .oma/sim2real/analyze_ankle_identifier_csv.py build/log/<csv_name>.csv`
- 约束：
  - 辨识时不运行 `run.sh`
  - 辨识时不能有其他模块或节点同时发布 `/joint_cmd`

## Round 2 下一步执行顺序

- 先补 `left_ankle_pitch_joint` 与 `left_ankle_roll_joint` 的悬空阶跃测试，沿用当前 `step_amplitude_rad = 0.015`
- `right_ankle_pitch_joint` 再补悬空对照，首个参数用 `kp=100, kd=0.8`
- 对每个 CSV 用 [analyze_ankle_identifier_csv.py](/Users/yumx/code/X1/agibot_x1_infer/.oma/sim2real/analyze_ankle_identifier_csv.py:1) 输出：
  - `command_step`
  - `actual_step`
  - `tracking_ratio`
  - `peak_overshoot`
  - `zero_crossing_count`
  - `response_class`
- 用同口径复查已有触地数据，按“先剔除振荡，再比较 tracking_ratio”重排
- 若悬空跟踪已接近 `1.0` 而触地下明显变软或振荡：
  - 转入接触耦合方向，重点看 `kd` 和 `lpf_conf.wc`
- 若悬空和触地都持续欠跟踪：
  - 继续向上扫描 `kp`
  - 不提前进入 `Round 3`

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
