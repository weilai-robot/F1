# 导航严格测试标准 (v2)

> 生成物: `scripts/nav_strict_gate.py` (CI 强制门禁) · `scripts/analyze_nav_trials.py` (观测)
> `scripts/nav_map_tools.py` (地图 A* 最优路径) · 场景: `scripts/nav_test_runner.py` SCENARIOS

## 判定结构

```
宽松层 (hmj 原有): success = ¬摔倒 ∧ 碰撞=0 ∧ 终点误差<0.35m      → CI 内继续输出
严格层 (本标准):   以下全部指标 × 全部场景 一票否决               → CI 最终 exit code
```

## 全局指标 (每场景必须满足)

| 类别 | 指标 | 阈值 | 说明 |
|------|------|------|------|
| P0 安全 | fall | =false | base_z<0.35m 或 roll/pitch>45° |
| P0 安全 | collisions | =0 | MuJoCo 接触计数 |
| P1 精度 | position_error_m | ≤0.20 | 终点位置误差 (GT) |
| P1 精度 | yaw_error_final_rad | ≤0.35 | 终点朝向误差 (GT 末帧中位, ≈20°) |
| P1 时效 | plan_time_s | ≤5.0 | goal→首次 cmd_vel |
| P1 定位 | drift_max_m | ≤0.20 | GT vs SLAM 逐帧漂移最大 |
| P1 定位 | drift_p95_m | ≤0.15 | 漂移 95 分位 |
| P2 平顺 | linear_jerk_rms | ≤5.5 | m/s³ (基线最优≈4.0) |
| P2 平顺 | angular_jerk_rms | ≤18.0 | rad/s³ (基线最优≈14.1) |
| P2 平顺 | direction_reversals_per_sec | ≤0.5 | 禁高频振荡 |
| P2 平顺 | vmin_m_s | ≥-0.05 | 禁明显倒退 |
| 有效性 | rtf_mean | ≥0.90 | RTF 过低则物理时序失真 |

## 动态指标 (按地图几何逐试验计算)

- **efficiency_optimal** = A*最优路径(机器人半径 0.25m 间隙约束) / 实际位移 ≥ **0.75**
  - 直线/实际 对有墙地图不公平 (基线 B 直线效率 0.477 但真实效率 0.81)
  - 起点取 timeseries 实际起点 — 顺序场景链也被公平评估
- **completion_time_s** ≤ 最优路径/0.15m/s + 15s (且 ≤0.75×timeout)

## 场景覆盖 (叠加)

| 场景 | 覆盖 | 特殊 |
|------|------|------|
| D_impassable | 不可达目标鲁棒性 | position_error ≤0.60 (goal 在墙内, 安全贴近即达标), 免效率 |

## CPU 预算 (pidstat 可用时)

aimrt_main ≤120% · nav2 ≤80% · fastlio ≤60% · lidar_bridge ≤40% (单核%)

## 场景集 v2 (修正墙内 goal/重复 goal)

| 场景 | goal | 修正原因 |
|------|------|---------|
| A_straight_5m | (5,0) | 不变: 隔断北门+玻璃门开口 (最优 8.44m) |
| B_obstacle_bypass | **(8,-3)** | 原 goal 与 A 完全相同 → A 成功即退化; 改为东区绕障 |
| C_narrow_passage | **(4.5,-3)** | 原 goal (5,-3) 在占据格内(墙), 几何不可达 |
| D_impassable | (5,3.2) 保留 | 故意不可达 → 改判定语义 (安全贴近≤0.6m) |
| E_long_distance | **(0,0)** | 原 (8,-3) 与 B 重复; 改全程返航 ~12.7m |
| F_return_trip | **(5,0)** | 复跑 A 路线, 重复性验证 |

## Harness 修复 (run 33830360449 复盘)

1. **A 冷启动失败根因**: bt_navigator 首个 action goal 响应因 DDS discovery 未就绪丢失
   (`Failed to send goal response (timeout)`), goal 被静默丢弃, 60s 无 cmd_vel。
   → 批量前 `warmup_nav_stack()` 零距离 goal 热身 + `resend_goal_if_lost()` 12s 看门狗重发(≤3次)
2. **z_voxels 25>16**: nav2 humble VoxelGrid 上限 16 层, 启动即 Error → 改 16
   (navigation@nav/dev-strict-v1)

## 阶段收紧路线

达标后按轮收紧: jerk 5.5→4.5/18→15 → drift 0.15→0.10 → vmean 0.15→0.20。
每轮在 git commit message 记录阈值变更与对应实测分布。
