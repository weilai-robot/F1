# AMP 训练、部署与 main 的 kp/kd 对比

日期：2026-10-13。当前部署基线 F1 `83433ee` / motion_control `dcb3128`，策略 `x1_amp_soup59c_50hz.onnx`。本次只分析参数，未调整任何控制增益。

已从 GitHub 更新 main 引用。F1 `origin/main` 为 `237d27b`，其 motion_control 指针为 **`59d321e`**，已含旧 `v46s` AMP。motion_control 自身的 `origin/main` 是 `12017c8`，没有 AMP；两者 stand、12DOF 和 14DOF 的增益相同。这里以 F1 main 实际固定的子模块版本为比较基准，未切换工作分支。

## 1. 标称参数结论

按关节名称解析、合并每个状态的控制器后，**当前真机 AMP、当前部署仿真 AMP、训练 X1_CFG 标称值、F1 main 的 AMP，29 个关节全部一致，增益差异为 0**。训练源码来自 `X1_29_AMP/roboparty_train/robolab/robolab/assets/robots/x1.py`，环境加载该 `X1_CFG`。

下表每格为 **kp / kd**。模型关节坐标中的 kp 单位为 Nm/rad，kd 为 Nm·s/rad。左右对称关节参数相同；旧策略列是实际状态的合并结果，包含未被 RL 控制器覆盖、仍由 PD 控制器保持的腰和上肢。

| 关节 | 训练标称 | 当前 AMP 部署 | main 12DOF 行走 | main 14DOF 腿臂行走 |
|---|---:|---:|---:|---:|
| 腰 yaw / roll | 120 / 4 | 120 / 4 | 500 / 5 | 500 / 5 |
| 腰 pitch | 150 / 5 | 150 / 5 | 500 / 5 | 500 / 5 |
| 左右肩 pitch | 40 / 2 | 40 / 2 | 80 / 1.5 | 20 / 3 |
| 左右肩 roll / yaw | 40 / 2 | 40 / 2 | 80 / 1.5 | 80 / 1.5 |
| 左右肘 pitch | 30 / 1.5 | 30 / 1.5 | 80 / 1.5 | 80 / 1.5 |
| 左右肘 yaw | 20 / 1 | 20 / 1 | 80 / 1.5 | 80 / 1.5 |
| 左右腕 pitch / roll | 15 / 1 | 15 / 1 | 80 / 1.5 | 80 / 1.5 |
| 左右髋 pitch | 120 / 4 | 120 / 4 | 30 / 3 | 60 / 6 |
| 左右髋 roll | 100 / 3.3 | 100 / 3.3 | 40 / 3 | 60 / 3 |
| 左右髋 yaw | 100 / 3.3 | 100 / 3.3 | 35 / 4 | 40 / 3 |
| 左右膝 pitch | 150 / 5 | 150 / 5 | 100 / 8 | 80 / 4 |
| 左右踝 pitch | 50 / 2.5 | 50 / 2.5 | 35 / 1.5 | 40 / 2 |
| 左右踝 roll | 50 / 2.5 | 50 / 2.5 | 30 / 1.5 | 30 / 2 |

F1 main 的 `rl_walk_amp` 也是本表的 AMP 值；其策略、关节序、软启动和滤波配置与当前版本不同，增益相同不代表整个控制闭环相同。

## 2. stand / zero / 静态方向测试使用另一组增益

当前与 main 的 stand/zero 按名称合并后的 kp/kd 相同：

| 部位 | stand / zero / 上肢静态测试基准 | AMP |
|---|---:|---:|
| 腰 yaw / roll | 500 / 5 | 120 / 4 |
| 腰 pitch | 500 / 5 | 150 / 5 |
| 肩 | 80 / 1.5 | 40 / 2 |
| 肘 pitch | 80 / 1.5 | 30 / 1.5 |
| 肘 yaw | 80 / 1.5 | 20 / 1 |
| 腕 | 80 / 1.5 | 15 / 1（L28 忽略此参数） |
| 髋 | 300 / 5 | pitch 120 / 4；roll/yaw 100 / 3.3 |
| 膝 | 300 / 5 | 150 / 5 |
| 踝 | 300 / 5 | 50 / 2.5（AMP 在软件中计算关节力矩） |

Guide 进入 `walk_amp` 后，`rl_walk_amp` 覆盖全部 29 个关节，先前 stand 的 500、300、80 不会继续叠加到 AMP。新上肢静态工具沿用 stand 增益，用于机械方向测试；其静态跟踪结果不能单独验证 AMP 动态增益是否合适。不同状态的目标角和控制方式也不同。

## 3. 配置相同与实际生效相同需要区分

训练用 `DelayedPDActuatorCfg`。标称 PD 的形式为 `tau = kp*(q_des-q) + kd*(dq_des-dq) + tau_ff`，包含力矩饱和和执行器指令延迟；训练此处 sim.dt=0.005 s、策略 50 Hz，执行器 min_delay=0/max_delay=2 对应 0～10 ms 的物理步延迟。定义见 [Isaac Lab 执行器文档](https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.actuators.html#isaaclab.actuators.DelayedPDActuator)。实机固件频率、内环滤波及真实延迟不能由这些标称值推断。

训练基础事件还在 startup 对 kp、kd 各自做 0.8～1.2 倍随机化。`X1_ACT_RAND=2` 分支可扩大到 0.7～1.3；soup 两个父模型的启动脚本没有显式设置此开关，不能仅据当前源码证明历史运行启用了扩大区间。标称值一致是合理的起点，不代表训练每个环境的瞬时增益固定，也不是任意调参的保证。

### 3.1 腰、髋、膝、肩、肘：驱动位置 PD

RLController 对这些关节发送位置目标、零目标速度、零前馈力矩和配置 kp/kd；传动层同时转换动作与反馈方向，kp/kd 保持正值。R52/R86 的当前主机 MIT 增益编码范围是 kp `[0,500]`、kd `[0,8]`，AMP 最大 150/5，全部在量程内。编码先限幅再量化，旧溢出/回绕已修正。正常量化步长约为 kp 0.1221、kd 0.001954，不是大幅减小增益的原因。

但电机固件的解码量程尚未取得现场证据。若主机和固件量程不同，物理增益仍会按比例偏离；修复回绕不等于校准固件。训练的腰/髋/膝 effort_limit_sim=180 Nm、肩肘=27 Nm，实际电机总 PD 力矩/电流限制也需要独立核对。

特别注意：SDK 中 R86 `toq_max=100`、R52 `toq_max=50` 是 **MIT 力矩字段的编码范围**。这些关节的前馈力矩字段为 0，不能只凭该范围就断言固件内部位置 PD 的总力矩上限是 100/50 Nm。

### 3.2 四个踝关节：软件关节 PD → 并联电机力矩

AMP 在 1 kHz 控制环按 `50*(q_des-q) - 2.5*dq` 计算关节力矩，裁剪到 ±80 Nm，再经并联传动映射到四个 R52 电机。位置/速度目标及下发 kp/kd 为 0；因此在 `/actuator_cmd` 中看到踝电机 kp=kd=0 是该控制方式的结果，不能认定为关节 PD 没生效。

关节 ±80 Nm 与 R52 电机力矩字段 ±50 Nm 位于不同坐标/传动空间。映射后若某电机力矩超过编码范围，会饱和并改变等效关节力矩；必须看实际姿态下的映射及电机指令。调试话题记录的是编码前数据，不能替代线上帧或固件反馈验证。此处数值上的 50/2.5 与训练一致，实际并联传动、速度反馈噪声和延迟仍可能改变闭环。

### 3.3 四个腕关节：L28 内部位置控制

真实 DCU 配置中四个腕执行器为 `POWER_FLOW_L28`。`PowerFlowL::SetMitCmd(float pos, float, float, float, float)` 只使用 pos，kp/kd 和前馈力矩参数未使用。训练的 15/1 是理想 PD 参数，实机此接口不设置 L28 内部增益；不能认定腕部的物理 PD 已与训练对齐。修改腕部 YAML 的 kp/kd 不能通过当前调用链改变其内环。

## 4. 是否合适：结论与判断依据

**当前数值适合作为本 soup 策略的部署基准，应先保留；尚不足以证明已完成真机闭环调优。** 当前参数既与训练标称一致，也与 F1 main 中原 AMP 一致，不存在本轮误把 main 站立增益整体套入 AMP 的问题。此前已确认的腰部符号错误与旧目标限位裁剪，应先与增益因素分开核验。

相比 main 12DOF，髋 pitch kp 为 4 倍，髋 roll 2.5 倍、髋 yaw 约 2.86 倍、膝 1.5 倍；腰与多数手臂则明显更软。这是不同策略的闭环配置差别，不能据旧策略能走就把整套 gain 复制给 AMP。

值得跟踪的是髋/膝的阻尼配比。例如膝从 main 12DOF 的 100/8 变为 AMP 150/5。对固定惯量、忽略耦合/接触/延迟的单关节线性近似，`kd/sqrt(kp)` 与阻尼比成比例；AMP 膝该指标约为旧值的 **0.51**，髋 yaw 约 **0.49**。这支持“相对旧 12DOF 更需检查过冲/振荡”的判断，不能据此直接断言欠阻尼或 kd=5 不合适。与 main 14DOF 比较，膝该指标约为 0.91，结论明显不同；比较基准不能混用。

建议先用修正方向、标零和限位之后的新 AMP 日志判断：

- 目标正常但反馈持续滞后/幅度不足：同时检查目标/力矩饱和、摩擦、传动与增益，不能直接增大 kp。
- 目标平滑但反馈反复过冲或抖动：先排除反馈噪声、延迟和错误方向，再固定 kp 做单项 kd 验证。若确认膝部需要更多阻尼，可将 150/5 → 150/5.5 → 150/6 作为小步候选；这些是待验证候选，不是本次已修改或已证明稳定的参数。
- 踝关节异常：先看软件 PD 的原始/限幅后力矩，以及映射后电机指令是否饱和。仅提高 kp 会放大该问题。
- 腕部异常：先核对 L28 的固件控制和位置响应，YAML 改 gain 不会改变当前内环。

旧腰日志来自修正方向/限位之前，不能证明修正后腰 150/5 的动态跟踪。没有新真机全身日志、固件量程或小幅响应数据，本次不判定一组“最佳真机 kp/kd”，也不修改部署增益。

## 5. 本次验证与源码位置

使用 Python AST 提取训练执行器的正则关节分组及标称参数，无需加载 Isaac Lab；读取 YAML 后按名称合并状态控制器，逐项核对全部 29 个关节。四套 AMP 标称增益映射完全相同，当前与 main 的 stand 增益映射也相同。

- 当前真机参数：[rl_x1.yaml](../motion_control/module/control_module/cfg/rl_x1.yaml)，控制器 `rl_walk_amp`。
- 部署仿真参数：[rl_x1_sim.yaml](../motion_control/module/control_module/cfg/rl_x1_sim.yaml)。
- F1 main 固定版本：[59d321e 的控制配置](https://github.com/weilai-robot/Humanoid_motion/blob/59d321e899539a856ed25323f30d969c2665e63c/module/control_module/cfg/rl_x1.yaml)。
- 软件踝 PD：[rl_controller.cc](../motion_control/module/control_module/src/rl_controller.cc)，`GetJointCmdData()`。
- 腕部位置接口：[power_flow.h](../motion_control/module/dcu_driver_module/xyber_controller/xyber_api/include/internal/power_flow.h)，`PowerFlowL`。
- 编码量程：[common_type.h](../motion_control/module/dcu_driver_module/xyber_controller/xyber_api/include/common_type.h)。
- 旧回绕问题与修复：[MIT 编码审计](amp_mit_encoding_audit.zh_CN.md)。
