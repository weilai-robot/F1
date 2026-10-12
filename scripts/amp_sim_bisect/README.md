# AMP 部署模型独立闭环验证

在 F1 仓库内运行 soup59c_50 的 NumPy 策略和部署 MuJoCo 模型，不需要训练仓库。
此脚本用于隔离策略、关节映射和模型；完整 ROS2/AimRT 链路使用
[服务器运行说明](../../doc/AMP仿真服务器运行.md)中的 `build/run_sim.sh`。

## 环境与运行

在 F1 根目录执行：

```bash
python3 -m venv /tmp/f1-amp-mj
/tmp/f1-amp-mj/bin/pip install mujoco==3.12.0 numpy

/tmp/f1-amp-mj/bin/python scripts/amp_sim_bisect/rollout_bisect.py \
  --ckpt scripts/amp_sim_bisect/soup59c_50.policy.npz \
  --repo-root . --robot deploy_flat --no-align \
  --spawn-pose stand --settle 0 --cmd 1 0 0 --duration 12 \
  --pd-every 1 --via-actuator --json /tmp/f1-amp-walk.json
```

零速测试将 `--cmd 1 0 0` 改成 `--cmd 0 0 0`。通过判据：无 `[FALL]`，
输出 `survived 600 steps = 12.00s of 600 commanded steps`，JSON 的 `fell` 为 `false`。
脚本当前检测到跌倒后也可能以退出码 0 结束，须检查上述结果。

默认策略 50 Hz、物理步长 1 ms、PD 每个物理步更新；`--via-actuator`
通过部署模型的 motor 通道施加 PD 力矩，并保留 actuator ctrlrange 限幅。
`--no-align` 保留 XML 的摩擦参数，但脚本仍设置关节被动 damping=1、armature=0.01；
它没有完整复刻 C++ 的 YAML 目标限位、传感器保护和 ROS2 调度。

## 2026-10-12 修正与验证

原便携回退顺序误用串行 GMR 顺序；策略实际使用训练 `x1.yaml` 的
`lab_dof_names`（Isaac Lab 交错顺序）。现已内置该顺序，按名称映射观测和动作；
若传入训练仓库，则校验其顺序是否一致。腿部调试输出也改为按关节名取索引。

修正前，固定同一策略和部署模型、零速、MuJoCo 3.12.0 时，错误顺序在 0.80 s 跌倒，
正确顺序通过 8 s。该对照已证明顺序错误足以造成跌倒；原先归因于平台/版本的结论需重新核验。

修正后的 F1 根目录零速和 1 m/s 指令测试均通过 12 s / 600 拍，开启 motor 通道及
ctrlrange 限幅。本地环境为 macOS arm64，完整 Linux `run_sim.sh` 集成仍需在服务器运行。
详细证据见 [Guide 跌倒分析](../../doc/amp_guide_failure_analysis.zh_CN.md)。
