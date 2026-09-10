# 运动控制 (motion_control) 仿真基础性能测试协议

> 用途: 导航侧需要量化 motion_control 在仿真中的基础运动能力 (直线/转弯/弧线/停止),
> 通过直接发送 vx/vy/wz 指令测量, 作为 Nav2 参数整定与安全边界设定的依据。
>
> 脚本: `scripts/run_loco_perf.sh` (一键编排) + `scripts/loco_perf_test.py` (执行器)
> 参考: 摔倒判据与 `scripts/nav_test_runner.py` 保持一致, 便于跨报告对比。

---

## 1. 被测链路与指令接口

```
测试脚本 ──/cmd_vel_limiter (Twist: vx, vy, wz)──▶ ControlModule (1kHz)
              │                                     └─ RL 策略 rl_walk_leg (100Hz, ONNX)
              │                                            └─ /joint_cmd (1kHz)
              ▼                                     SimModule ── mj_step (1ms 物理步长)
   /mujoco/ground_truth ◀─────────────────────────────────────┘
   (sim_time, x, y, z, roll, pitch, yaw, rtf, collisions, cum_dist)
```

- 速度指令入口: `/cmd_vel_limiter` (external ROS2 publish, 50Hz 持续发送, 变更立即生效)。
- 状态机: 仿真初始 `stand`; 测试先发 `/stand_mode` 再持续 2s 发 `/walk_mode` 进入
  `walk_leg` (控制端有 1s 节流, 单次 publish 会丢, 必须持续发)。
- 指令范围: 摇杆限幅盒 (vx±0.5, vy±0.3, wz±0.5) 仅约束手柄路径; 直发
  `/cmd_vel_limiter` 无软件截断, 扫描上限由测试脚本给定 (默认 vx≤1.4, wz≤1.4)。
- 真值只有 `/mujoco/ground_truth` 一条流 (含位置/姿态), 平地场景默认配置不含此话题,
  性能测试使用专用场景配置 `x1_cfg_sim_perf.yaml` + `sim_x1_perf.yaml` (平地 + GT)。

## 2. 测试环境

| 项 | 值 | 说明 |
|----|-----|------|
| 场景 | `xyber_x1_flat.xml` (平地, 无墙) | 避免障碍物干扰极限速度判定 |
| 配置 | `cfg/x1_cfg_sim_perf.yaml` | 由 `run_loco_perf.sh` 自动生成/由构建安装 |
| 启动 | `cd build && ./aimrt_main --cfg_file_path=./cfg/x1_cfg_sim_perf.yaml` | tmux 窗口 `f1_loco_perf` |
| 数据 | `/mujoco/ground_truth` (~1kHz) | 每个 `/joint_cmd` 步进一帧并发布 |
| 步进保真 | 档位保持按 **sim 时间** 计时 | RTF<1 时 wall 时长自动拉长, 不影响口径 |

**摔倒判据** (与 nav_test_runner 一致): `z < 0.35m` 或 `|roll| > 45°` 或 `|pitch| > 45°`。
单档扫描中一旦摔倒立即中止扫描并记录摔倒档位 (此时机器人倒地, 后续测试需重启仿真,
`run_loco_perf.sh` 默认每项测试前自动重启)。

## 3. 四项测试定义

### 3.1 直线 (straight)

- 指令序列: `vx = 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4` m/s 逐档递增, 每档保持 6s, 摔倒即止。
- 测量段: 档位开始后 0.5s ~ 档位结束; 稳态窗口: 测量段最后 3s。
- 指标:
  - **速度达成率** `η = 稳态窗口平均速度 / 指令速度`
  - **是否走直线**: 航向漂移率 ≤ 3°/m **且** 偏移率 ≤ 5%
  - **偏移率** `= max|横向偏差| / 行驶路程` (横向偏差相对测量段起始 0.25s 中位位姿的初始航向线, 左正)
  - **航向漂移率** `= |Δyaw| / 行驶路程` (°/m)
- 输出: **最大速度(不摔倒)**、最大跟踪速度 (η≥50%)、各档达成率/偏移率/航向漂移率。

### 3.2 原地转弯 (turn)

- 指令序列: `wz = 0.2, 0.4, ..., 1.4` rad/s 逐档递增 (默认逆时针), 每档 6s。
- 指标:
  - **转速达成率** `η = 稳态窗口平均 yaw rate / 指令角速度` (yaw rate 由 GT 姿态差分)
  - **是否偏移原地**: 最大位移 ≤ 0.15m **且** 单位角度偏移 ≤ 0.05 m/rad
  - **偏移率** `= max|位移| / |总转角|` (m/rad); 另报最大位移 d_max (m)
- 输出: **最大角速度(不摔倒)**、最大跟踪角速度 (η≥50%)、各档偏移指标。

### 3.3 弧线 (arc)

- 指令序列: (vx, wz) 成对递增, 默认恒定半径 R=1m: (0.2,0.2) → (1.0,1.0)。
- 指标:
  - **达成率** `η_v, η_w` (同上口径); 方向一致性 (实测转向符号 = 指令符号)
  - **半径误差** `= |R_fit − R_cmd| / R_cmd` (Kasa 圆拟合, R_cmd = vx/wz)
  - **轨迹残差 RMS** = 各点到拟合圆距离的 RMS (量化"走偏")
- 输出: **最大执行速度(不摔倒)**、该档达成率、半径误差、残差。

### 3.4 停止 (stop)

- 指令序列 (每次重复): 加速到 `0.4 m/s` 保持 3.5s → 发布零速度 → 观察 3.5s; 重复 5 次。
- 时刻对齐: 零指令的 wall 时刻经 `(wall, sim)` 局部线性回归映射到 sim 时间 `t0`
  (消除传输延迟, 精度 ~±10ms)。
- 指标:
  - **停止时间** `= t0 → 速度首次 < 0.05 m/s 且持续 0.1s`
  - **最大停止减速度** = 平滑速度微分的峰值 (t0 ~ t_stop 区间); 另报平均减速度 `v0/t_stop`
  - **停止距离** = t0 ~ t_stop 行驶路程
- 输出: 每次明细 + mean/max 统计 (支持 `--stop-speeds "0.4,0.6,0.8"` 多初速)。

## 4. 运行方法

```bash
# 一键: 全部 4 项 (每项前自动重启仿真保证初始状态干净)
./scripts/run_loco_perf.sh

# 单项 / 自定义档位 / 自定义保持时间
./scripts/run_loco_perf.sh --tests straight --levels "0.2,0.4,0.6,0.8"
./scripts/run_loco_perf.sh --tests stop --hold 4

# 附着到已在运行的仿真 (仿真需已发布 /mujoco/ground_truth)
./scripts/run_loco_perf.sh --no-sim --tests turn

# 单项执行器直用 (仿真需已运行且进入 walk 模式)
python3 scripts/loco_perf_test.py --test arc --levels "0.3:0.3,0.5:0.5" --report-dir reports
python3 scripts/loco_perf_test.py --selftest        # 指标核心自检 (无需 ROS/仿真)
```

> 注意: `motion_control` 重新构建后 `sim_x1_perf.yaml` / `x1_cfg_sim_perf.yaml` 随构建安装;
> 未重新构建的机器上, `run_loco_perf.sh` 会自动在 `build/` 生成配置副本。

## 5. 产出文件

| 文件 | 内容 |
|------|------|
| `reports/loco_<test>_<时间戳>.md` | 可读报告: 总览 + 分级明细表 |
| `reports/loco_<test>_<时间戳>.json` | 机读: 各档指标 + 降采样曲线 + summary |
| `reports/loco_<test>_<时间戳>.gt.csv` | 原始 GT 逐帧 (wall, sim_time, x, y, ...) |
| `reports/loco_<test>_<时间戳>.cmd.csv` | 指令时间线 (wall, vx, vy, wz) |

退出码: `0`=完成; `2`=完成但出现摔倒; `3`=中断/前置条件不满足。

## 6. 精度与假设

1. **GT 频率**: 设计上每控制周期一帧 (~1kHz), 实际以报告 `meta.gt_rate_hz` 为准;
   若 < 200Hz, 停止段精度下降, 建议排查。
2. **速度/减速度获取**: GT 位置中心差分 (51ms 平滑); 停止时刻判定另有 0.1s 持续条件,
   对单帧噪声不敏感。
3. **wall→sim 映射**: 局部线性回归, 停止时刻对齐误差 ~±10ms 量级。
4. **确定性**: 仿真默认无控制噪声; 同一档位重复测试结果应基本一致; 差异大则检查 RTF。
5. **平面无限大**: 测试中机器人跑偏无碰撞风险; 但 `collisions` 字段仍记录 (应恒为 0)。
6. **单方向**: 默认前进/逆时针; 反向验证可用负值档位 (如 `--levels "-0.2,-0.4"`, 转向试点)。

## 7. 判定阈值说明 (重要)

指标中的判定阈值 (直线 3°/m / 5%, 原地 0.15m / 0.05 m/rad, 弧线 25% / 0.10m) 为
**初始建议值, 待首轮基线数据校准**; 全部定义在 `scripts/loco_perf_test.py` 顶部
`V_*` 常量, 校准只需改常量。报告同时给出原始数值, 阈值调整不损失信息。

## 8. 已知限制与后续扩展

- 扫描为**单次升序**, 摔倒档位即上界; 如需精确边界可手动补测中间档。
- 未做: vy 横移扫描、指令阶跃响应 (上升时间/超调)、控制噪声鲁棒性、真机对照。
  上述可作为二期项, 复用现有 runner 框架 (只增分析函数)。
- 摔倒后必须重启仿真 (无在位复位指令); 一键脚本已自动处理。

## 9. 结果使用建议 (供导航侧参考)

- **Nav2 速度上限**: 建议取"最大不摔倒速度 × 0.7~0.8"作为 `max_vel_x` 的安全裕度起点。
- **转弯约束**: MPPI 的 wz 上限参考"最大跟踪角速度"档位, 而非极限档。
- **停止**: 0.4m/s 停止时间/距离直接用于前向安全距离与 `controller` 减速参数校核。
