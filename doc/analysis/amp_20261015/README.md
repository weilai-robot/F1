# AMP 2026-10-15 数据分析准备（v70b m16000b + v68 辨识 PD）

## 数据三层对照

| 层 | 来源 | 位置 |
|---|---|---|
| 新真机 | 人工工单回传 `walk_diag_*.csv`（v70b m16000b + 新 PD） | 待上传 |
| 新 sim 基线 | v70b m16000b sim2sim（v68 PD + 辨识 armature）→ 部署格式合成 | `sim_baseline/walk_diag_sim_v70b_walk10.csv`、`walk05.csv` |
| 旧真机基线 | 2026-10-13 soup59c + 旧 PD 两轮 | `../amp_20261013/metrics.json` |

sim 基线由 `scripts/gen_sim_walk_diag.py` 生成：从 rollout npz 因果重放策略
（关节序按 lab/policy 置换，与分析器重放验证 mae 4.2e-07 ≈ 10-13 真机验证量级）。
合成字段约定：effort=tau_des（指令力矩）、imu_accel=速度差分近似、actuator_* 列缺省。

## 真机数据到达后的运行方式

```bash
# 单文件多轮分析（--policy 用于重放验证"上腿的就是 m16000b"）
python3 scripts/analyze_amp_walk_logs.py <新csv...> \
  --policy /Users/yumx/code/X1_29_AMP/acceptance/v70_eval/m16000b.policy.npz \
  --out doc/analysis/amp_20261015/real_metrics

# sim 基线指标（已可复算）
python3 scripts/analyze_amp_walk_logs.py doc/analysis/amp_20261015/sim_baseline/*.csv \
  --policy /Users/yumx/code/X1_29_AMP/acceptance/v70_eval/m16000b.policy.npz \
  --out doc/analysis/amp_20261015/sim_metrics
```

## 报告要点（对照 10-13 结论）

1. **重放验证**：mae 应 ~1e-6（模型/观测排列/归一化一致）；若大，先查是否跑错模型
2. **反馈健康**：freeze_groups——左臂 7 关节是否仍成组冻结（10-13 的 P0 问题）
3. **步态周期**：cycles 周期 CV（10-13 实机 7–17% vs 仿真 1.6–1.7%）；同侧膝峰间隔
4. **左右对称**：bilateral_correlations、左右膝/髋 ROM（10-13 左膝 55° vs 右 36°）
5. **跟踪与相位**：mae_rad + 最优时移（10-13 髋 120ms/膝 80–100ms 表观滞后）
6. **摆幅比**：实际/目标 ROM 比（10-13 左膝 1.27–1.28 超调）
7. **新 PD 生效验证**：kd=8 下跟踪误差应明显缩小（辨识目标 ζ 0.7）
8. **摆臂**：肩/肘 action 与 pos 摆幅（v70 arm-fold 修复验证；右前臂内收 <15° 门限）

## 基线快照（v70b sim walk1.0，供快速对照）

- 左膝跟踪 MAE 0.0750 rad（4.3°）| 目标-实际 bias −0.0096 rad
- 新真机若显著 > 此值（10-13 为 0.255 rad / 14.6°），说明新 PD + 新模型确有改善或暴露新问题
- 详细分位数见 sim_metrics（数据到达后统一复算入报告）
