# 真机 kp/kd 变小：MIT 编码排查

日期：2026-10-12。排查基线：F1 `00e8157`，motion_control `8e6eedb`。结论来自配置、下发链路和生产编码器的离线报文实验，未连接真机。

## 结论

旧版 R52/R86 驱动存在确定的编码溢出：浮点值未限幅就转成整数，随后打包到固定宽度的字节。kp/kd 只有 12 位，超范围后高位被丢掉，效果类似对整数码取余 4096；代码没有对浮点 kp/kd 直接使用 `%`。kp 和力矩的高位还可能串入相邻字段。

**当前仓库真机 `rl_x1.yaml` 的所有控制器 kp/kd 都在驱动默认量程内，不能将本次真机跌倒直接归因于增益溢出。** 仿真配置确实有 kp=700、kd=10，若现场把这些值用于真机，旧驱动会把它们编码成小值。还需核对现场实际加载配置，以及电机固件量程。

本次修复在浮点转整数前按各字段的 MIT 量程限幅，并在两个共享字节中分别限制高半字节。未调整策略模型、控制器增益或驱动默认量程。超范围输入现在饱和到端点，不再回绕；饱和并不意味着电机能够实现超出其协议量程的指令。

## 1. 下发链路与默认量程

`ControlModule` → `JointCommand.stiffness/damping` → `DcuDriverModule::JointCmdCallback` → 传动映射 → `XyberController::SetMitCmd` → `Dcu::SetMitCmd` → `PowerFlowR::SetMitCmd`。

相关代码：

- [关节指令接收和电机下发](../motion_control/module/dcu_driver_module/src/dcu_driver_module.cc)
- [R52/R86 默认 MIT 量程](../motion_control/module/dcu_driver_module/xyber_controller/xyber_api/include/common_type.h)
- [创建电机时使用默认量程](../motion_control/module/dcu_driver_module/xyber_controller/xyber_api/src/xyber_controller.cpp)
- [浮点量化和字节打包](../motion_control/module/dcu_driver_module/xyber_controller/xyber_api/src/power_flow.cpp)

| 字段 | R52 | R86 | 位宽 |
|---|---|---|---|
| kp | 0～500 | 0～500 | 12 |
| kd | 0～8 | 0～8 | 12 |
| 力矩 | −50～50 Nm | −100～100 Nm | 12 |
| 位置 | −2π～2π rad | −2π～2π rad | 16 |
| 速度 | −4π～4π rad/s | −4π～4π rad/s | 12 |

旧量化公式为 `int((x - min) * 4095 / (max - min))`（12 位字段）。例如 kp=700 得到整数码 5733，打包后的 kp 码只剩 1637，按驱动默认量程解码约为 199.8779。

默认量程是主机 SDK 的假设，尚无本次真机固件配置的证据。部署调用链没有调用 `SetMitParam` 来覆盖量程；该接口在 R 驱动内只修改主机成员变量，也不自动设置电机固件。如果电机和主机的量程不同，即使没有溢出也会得到错误的物理增益。例如主机按 kd 最大 8 编码、电机按最大 5 解码，则实际 kd 约为输入的 5/8。这是量程不一致造成的比例误差，需在现场独立核对。

## 2. 原始报文复现与修复后结果

实验直接调用生产 `PowerFlowR::SetMitCmd`，取得 8 字节发送缓冲区，再独立拆出各字段并按默认量程解码；不是读取编码前的 ROS 指令。

| 输入 | 旧编码解码值 | 修复后解码值 |
|---|---|---|
| kp=500 | 500 | 500 |
| kp=600 | 99.8779 | 500 |
| kp=700 | 199.8779 | 500 |
| kp=500.2 | 0 | 500 |
| kd=5 | 4.99927 | 4.99927 |
| kd=10 | 1.99658 | 8 |
| kd=8.01 | 0.007814 | 8 |
| R52 力矩 +80 Nm，kd=0 | 力矩 −20.0366 Nm，kd≈0.001954 | 力矩 +50 Nm，kd=0 |
| R52 力矩 −80 Nm，kd=0 | 力矩 +20.0366 Nm，kd≈0.029304 | 力矩 −50 Nm，kd=0 |

正常范围内的少量差值是量化：kp 一步约 0.1221，kd 一步约 0.001954，不属于回绕。旧打包表达式 `vel_low << 4 | kp_code >> 8`、`kd_low << 4 | torque_code >> 8` 没有隔离高半字节，所以溢出除了影响本字段，还可能污染速度或 kd。

## 3. 当前配置是否触发

真机入口 [x1_cfg.yaml](../motion_control/install/linux/bin/cfg/x1_cfg.yaml) 加载 [rl_x1.yaml](../motion_control/module/control_module/cfg/rl_x1.yaml)，仿真另用 [rl_x1_sim.yaml](../motion_control/module/control_module/cfg/rl_x1_sim.yaml)。控制器按状态列表顺序按关节名称合并，后者覆盖前者。真机 AMP 控制器覆盖全部 29 个关节。

| 真机控制器 | 最大 kp | 最大 kd | 增益超范围 |
|---|---|---|---|
| pd_idle | 0 | 0 | 无 |
| pd_keep / pd_zero | 500 | 5 | 无 |
| pd_stand | 300 | 5 | 无 |
| pd_plan | 80 | 1.5 | 无 |
| rl_walk_leg | 100 | 8 | 无 |
| rl_walk_leg_shoulder | 80 | 6 | 无 |
| rl_walk_amp | 150 | 5 | 无 |

仿真配置中，`pd_keep/pd_zero` 的腰部三个关节 kp=700；`rl_walk_leg` 的左右膝 kd=10。仿真由 SimModule 执行，不经过真实 DCU 的 MIT 打包。将这份配置或这些增益用于旧真机驱动会触发上述回绕。

另有两种“配置有增益、电机端没有”的情况，不能与溢出混为一谈：

- **AMP 四个踝关节**：软件以 kp=50、kd=2.5 计算关节力矩，再经并联机构映射到四个 R52 电机。关节命令的 kp/kd 被主动设为 0，因此对应电机 kp/kd 为 0 是既有设计。关节力矩上限 ±80 Nm 与电机 MIT 量程 ±50 Nm 不在同一空间；需要看映射后电机指令是否饱和，不能仅凭关节上限判断。旧编码若收到超范围电机力矩，会发生表中的回绕；本次修复消除了该编码风险，但尚未证明此次真机首拍触发了它。
- **四个腕部 L28 电机**：`PowerFlowL::SetMitCmd` 只写位置，kp/kd 参数未使用。AMP 腕部配置的 kp=15、kd=1 不会通过该接口改变电机增益，需要结合 L28 固件位置控制器理解实际行为。

`/actuator_cmd` 调试话题发布的是 `actuator_data_space_.cmd` 中编码前的值。它可以显示 kp=700 或力矩=80，但不能证明线上帧传输了相同物理值，也不会显示本次编码器内部的限幅结果。

## 4. 验证与服务器使用

新增无硬件依赖测试，直接编译生产编码器。覆盖 R52/R86 五个字段的上下限、极大有限值、字段隔离、具体溢出案例，以及 2002 组范围内报文的量化误差。命令：

```bash
cd ~/code/F1
bash scripts/test_mit_encoding.sh
```

本机测试通过，并通过 UndefinedBehaviorSanitizer / float-cast-overflow 检查。旧编码器运行同一回归测试失败（`positive overflow must saturate`）。另将 R52/R86 各 10000 帧、五个字段分别采样的正常量程报文与旧编码器逐字节对照，20000 帧全部一致，报文流 SHA256 为 `3ce5981d992ad34a7fbcb425a8b79064157b76f64a494db9707b30e77ecabf09`。测试仅验证主机编码，不代替电机固件量程、实机力矩和完整 Linux 集成验证。

拉取修复后需要重新编译；只修改 YAML 或继续运行旧二进制不会修复编码器：

```bash
cd ~/code/F1
git pull --ff-only origin feat/amp-deploy
git submodule update --init --recursive
bash scripts/test_mit_encoding.sh
bash scripts/build.sh
cd build
bash run.sh
```

启动前确认运行目录 `cfg/control_module/rl_x1.yaml` 和电机固件的 MIT 量程。保留现场网卡、DCU 拓扑和标零补偿；构建安装会更新仓库配置。当前真机默认 Guide 切入后立即给固定 1 m/s 指令，修复报文编码尚不能证明整机步态已通过真机验证。
