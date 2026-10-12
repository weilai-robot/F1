# AMP 部署侧仿真二分 harness（便携版）

用途：在**任意机器**上用训练侧同款 rollout 验证 AMP 策略（soup59c_50）在
F1 部署模型上的行为，绕开 AimRT 集成层。当前用于定位"mac 走 / linux 倒"
的平台差异。

## 环境要求

```bash
python3 -m venv /tmp/mjv && /tmp/mjv/bin/pip install mujoco==3.12.0 numpy
```

## 运行（在 F1 仓库根目录）

```bash
# 关键格: F1 部署模型 + 完整 flat 场景 + vendor 动力学 + 浅蹲出生 + settle0
/tmp/mjv/bin/python scripts/amp_sim_bisect/rollout_bisect.py \
  --ckpt scripts/amp_sim_bisect/soup59c_50.policy.npz \
  --repo-root . \
  --robot deploy_flat --no-align --spawn-pose stand --settle 0 --duration 8

# 判据: survived N of N = 走通; [FALL] = 倒
# 对照格(mac 上全走通, 见经验库): 加/去 --no-align、--settle 0.5/5、--spawn-pose default
```

## 历史结论（macOS arm64, mujoco 3.12.0 wheel）

- 16+ 配置组合全部 600/600 走通（模型/被动动力学/地板/执行通道/ctrlrange/PD频率/出生态）
- 同配置锁 mujoco==3.1.3 → 全倒（33 拍, yaw 发散）→ 部署 sim 3.1.3 假阴性根因
- AimRT(Linux C 库 3.12.0, 干净重编)仍倒 → 待本 harness 在 linux 上跑出分界
