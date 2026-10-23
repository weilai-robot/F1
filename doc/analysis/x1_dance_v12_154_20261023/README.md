# v12 154 维舞蹈部署验证（2026-10-23）

完整配置、构建和操作方式见 [154 维舞蹈部署](../../BeyondMimic_X1_154维舞蹈部署.md)。

`imu_yaw0.json`、`imu_yaw0p8.json`、`imu_yawm1p2.json` 是新部署资产与归一化、1 kHz / 50 Hz ZOH、准备位自动接管的 Python/MuJoCo 结果。姿态采用生产 C++ `ComputeX1DanceAnchor`，不使用身体位置/线速度估计器；记录的位置误差来自独立仿真真值。三组完成 647 帧、12940 个播放物理步。仿真从已站立状态开始，未模拟冷启动 A/B 过程。

`legacy_v11_yaw0.json` 是相同仿真读取器对旧 160 维资产与原配置的回归结果，完成 647 帧。`validation.json` 记录已执行检查、未执行检查及来源 SHA256。

本地独立 C++ 检查命令（在 motion_control 目录）：

```bash
clang++ -std=c++20 -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer \
  -Imodule/control_module/include module/control_module/test/dance_state_adapter_check.cc \
  -o /tmp/f1_v12_adapter_check
/tmp/f1_v12_adapter_check
clang++ -std=c++20 -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer -pthread \
  -Imodule/control_module/include module/control_module/test/dance_diagnostics_check.cc \
  -o /tmp/f1_v12_diagnostics_check
/tmp/f1_v12_diagnostics_check
```

这些检查和仿真没有运行完整 C++/ROS 控制器、EtherCAT/DCU 或电机。新增控制器 gtest 与整个 Ubuntu 构建待在具备依赖的目标环境执行。
