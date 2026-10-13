# 上半身 direction 测试链路审计

日期：2026-10-13。审查基线：F1 `154d04b` / motion_control `dcb3128`。用户反馈多个右臂关节测试均不动；本次检查腰、左臂和右臂，保持现有方向、增益和测试角。

## 当前结论

没有发现生成配置、PD 插值、按名称合并、传动映射或 SDK CAN 缓冲选择中存在整体跳过右臂的分支。当前默认配置覆盖全部 17 个上半身关节，腰静态 kp/kd=500/5，左右肩肘及腕指令均为 80/1.5；R52/R86 增益在编码范围内。SDK 正常 SetMitCmd 将对应通道发送掩码设为 0xFF，包含该通道各 CAN ID。

多个右臂关节共同不响应，优先检查 body DCU 的 channel_2，以及现场配置、电机使能、控制模式和反馈；这是一条排查优先级，尚不是已确认的实机根因。方向参数是 ±1，仅改变符号；方向错误通常会反向运动，不足以单独解释不同轴的整条右臂都没有响应。

## 已修复的代码问题

1. DCU 的指令缓存写入、SDK 下发和诊断缓存读取原来没有被同一互斥锁完整保护。反馈发布线程同时更新传动使用的电机状态，也存在并发读写。现在用 rw_mtx_ 保护共享状态/指令的读写和传动，并在锁内取得诊断快照，锁外发布话题。SDK 反馈先读入发布线程自身的消息对象。该修复消除数据竞争，但没有现场证据证明它导致整条右臂不动；需要复核实际调度周期。

2. L28 腕电机把 CAN 字节槽强转成包含 float 的 L28Data 指针。DCU TX 每通道 65 字节且开头有 1 字节控制码，槽地址不保证 4 字节对齐。实际 SDK 路由测试通过 UBSan 复现了未对齐访问。已改用 memcpy 读写 float，保持原生字节序、8 字节载荷和默认电流不变。此缺陷涉及两侧腕电机，不能解释 R52/R86 肩肘全部不动。

3. 旧测试客户端只核对 /joint_cmd 的位置，没有检查 kp/kd、/actuator_cmd 和电机位移。现在逐项验证关节和传动后电机的目标及增益，显示实际位移，并新增 n=完全不动及只读 diagnose。准备阶段检查启用状态、遗漏电机、重复 CAN 地址和零静态 kp。

## 仍需现场核实的固件链路

InitDcu 只调用 EnableActuator 和 SetMitCmd，没有显式 SetMode(MODE_MIT)。因此 R52/R86 的既有固件模式是部署前提。PowerFlowR 未实现 GetMode 读回，继承的默认值为 MODE_CURRENT；Dcu::SetMode 等待 GetMode 的结果不能作为可靠模式确认。不能为了排查直接把这个默认值当成现场模式，或直接加入等待式 SetMode 调用。需要按电机固件工具核对右臂是否接受 MIT 指令。

EnableActuator 对 R52/R86 等待 STATE_ENABLE 确认，失败会导致 DCU 初始化失败；对 L28 则直接返回 true，并没有相同的使能确认。新启动日志打印每个电机的路由和该接口返回结果，不能把 L28 的 true 当作物理使能 ACK。SDK 的 GetErrorString 当前也不能提供可信的详细电机故障解释。

/actuator_cmd 是编码前缓存，/actuator_states 是 SDK 接收缓存。其上层话题持续更新不代表每个 CAN 电机反馈都新鲜，也不证明固件接收和执行了指令。需要用动作期间的反馈变化、启用日志及现场 CAN/固件状态定位下一层。

## 默认配置中的全部上半身路由

表中关节和电机名称省略 _joint / _actuator 后缀。目标为默认 0.10 rad 测试、当前零偏为 0 时的原生电机目标。肩 roll 测试使用负关节增量。现场有其他标零偏移或网络配置时，以生成清单为准。

| 关节 | 电机 | DCU / ECAT | 通道 / CAN | direction | 关节增量 | 电机目标 |
|---|---|---|---|---:|---:|---:|
| lumbar_yaw | lumbar_yaw | hip / 2 | 3 / 1 | +1 | +0.10 | +0.10 |
| lumbar_roll | lumbar_left | body / 1 | 3 / 1 | +1 | +0.10 | +0.10 |
| lumbar_pitch | lumbar_right | body / 1 | 3 / 2 | -1 | +0.10 | -0.10 |
| left_shoulder_pitch | left_shoulder_pitch | body / 1 | 1 / 1 | +1 | +0.10 | +0.10 |
| left_shoulder_roll | left_shoulder_roll | body / 1 | 1 / 2 | -1 | -0.10 | +0.10 |
| left_shoulder_yaw | left_shoulder_yaw | body / 1 | 1 / 3 | +1 | +0.10 | +0.10 |
| left_elbow_pitch | left_elbow_pitch | body / 1 | 1 / 4 | +1 | +0.10 | +0.10 |
| left_elbow_yaw | left_elbow_yaw | body / 1 | 1 / 5 | +1 | +0.10 | +0.10 |
| left_wrist_pitch | left_wrist_back | body / 1 | 1 / 7 | +1 | +0.10 | +0.10 |
| left_wrist_roll | left_wrist_front | body / 1 | 1 / 6 | +1 | +0.10 | +0.10 |
| right_shoulder_pitch | right_shoulder_pitch | body / 1 | 2 / 1 | -1 | +0.10 | -0.10 |
| right_shoulder_roll | right_shoulder_roll | body / 1 | 2 / 2 | +1 | -0.10 | -0.10 |
| right_shoulder_yaw | right_shoulder_yaw | body / 1 | 2 / 3 | +1 | +0.10 | +0.10 |
| right_elbow_pitch | right_elbow_pitch | body / 1 | 2 / 4 | -1 | +0.10 | -0.10 |
| right_elbow_yaw | right_elbow_yaw | body / 1 | 2 / 5 | +1 | +0.10 | +0.10 |
| right_wrist_pitch | right_wrist_back | body / 1 | 2 / 6 | +1 | +0.10 | +0.10 |
| right_wrist_roll | right_wrist_front | body / 1 | 2 / 7 | +1 | +0.10 | +0.10 |

## 真机定位顺序

更新子模块并重新构建后，停止旧测试，运行 start 生成版本 2 配置。先以默认测试角单测 right_shoulder_pitch_joint，观察工具打印的关节目标、原生电机目标和位移。可保持该目标，在另一终端运行 diagnose 汇总腰与左右臂。

| 观察 | 排查位置 |
|---|---|
| 所选关节目标未到达 | 模式事件、状态机、当前进程是否加载测试配置；会报告关节目标错误 |
| 关节目标到达，但电机目标或 kp/kd 不匹配 | DCU 订阅、实际传动配置、缓存链路；会报告具体电机 |
| 电机指令匹配，动作时电机反馈仍无位移 | 编码后总线、channel_2 接线、使能、MIT 模式、固件/反馈或机械阻碍；不是方向结论 |
| 电机有位移，但动作部位错误 | CAN ID/接线/电机命名和绑定；记录 w |
| 电机有位移，机械方向相反 | 才进入 direction/标零排查；记录 r |

## 验证范围

15 项离线测试通过，其中模拟客户端对全部 17 个关节逐项运行；覆盖错误映射、错误增益、右臂无运动、断流和竞争发布者。实际 C++ SDK 测试对全部 17 个配置路由逐一更新：只允许目标电机的 8 字节槽变化，其他通道/电机不变，校验位置/增益解码及接收槽；修复后 UBSan 无错误。完整 Linux AimRT/ROS2 构建、控制周期及实机动作尚未在本机验证。

执行步骤：[上肢静态方向测试](上肢静态方向测试.md)。
