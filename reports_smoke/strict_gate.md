# 严格测试标准门禁 (强制)

- 通过: 1/2
- 详细指标判定:

| 指标 | 阈值 | 实测 | 判定 | 裕量 |
|------|------|------|------|------|
| **A_straight_5m** | | | ✅ | |
| &nbsp;&nbsp;fall | == False | False | ✅ |  |
| &nbsp;&nbsp;collisions | == 0 | 0 | ✅ |  |
| &nbsp;&nbsp;position_error_m | ≤ 0.2 | 0.12 | ✅ | -0.08 |
| &nbsp;&nbsp;yaw_error_final_rad | ≤ 0.35 | 0.01 | ✅ | -0.34 |
| &nbsp;&nbsp;plan_time_s | ≤ 5.0 | 2.2 | ✅ | -2.8 |
| &nbsp;&nbsp;drift_max_m | ≤ 0.15 | 0.05 | ✅ | -0.1 |
| &nbsp;&nbsp;drift_p95_m | ≤ 0.1 | 0.02 | ✅ | -0.08 |
| &nbsp;&nbsp;linear_jerk_rms | ≤ 1.5 | 0.4 | ✅ | -1.1 |
| &nbsp;&nbsp;angular_jerk_rms | ≤ 4.0 | 0.8 | ✅ | -3.2 |
| &nbsp;&nbsp;direction_reversals_per_sec | ≤ 0.5 | 0.1 | ✅ | -0.4 |
| &nbsp;&nbsp;vmin_m_s | ≥ -0.05 | 0.0 | ✅ | +0.05 |
| &nbsp;&nbsp;rtf_mean | ≥ 0.9 | 0.96 | ✅ | +0.06 |
| &nbsp;&nbsp;path_efficiency | ≥ 0.9 | 0.95 | ✅ | +0.05 |
| &nbsp;&nbsp;completion_time_s | ≤ 30.0 | 22.5 | ✅ | -7.5 |
| **C_narrow_passage** | | | ❌ collisions,position_error_m,yaw_error_final_rad(MISSING),plan_time_s,drift_max_m,drift_p95_m,linear_jerk_rms,angular_jerk_rms,direction_reversals_per_sec,vmin_m_s,rtf_mean,path_efficiency,completion_time_s | |
| &nbsp;&nbsp;fall | == False | False | ✅ |  |
| &nbsp;&nbsp;collisions | == 0 | 1 | ❌ |  |
| &nbsp;&nbsp;position_error_m | ≤ 0.2 | 0.5 | ❌ | +0.3 |
| &nbsp;&nbsp;yaw_error_final_rad | ≤ 0.35 | N/A | ❌ |  |
| &nbsp;&nbsp;plan_time_s | ≤ 5.0 | 8.0 | ❌ | +3.0 |
| &nbsp;&nbsp;drift_max_m | ≤ 0.15 | 0.4 | ❌ | +0.25 |
| &nbsp;&nbsp;drift_p95_m | ≤ 0.1 | 0.2 | ❌ | +0.1 |
| &nbsp;&nbsp;linear_jerk_rms | ≤ 1.5 | 3.0 | ❌ | +1.5 |
| &nbsp;&nbsp;angular_jerk_rms | ≤ 4.0 | 9.0 | ❌ | +5.0 |
| &nbsp;&nbsp;direction_reversals_per_sec | ≤ 0.5 | 1.4 | ❌ | +0.9 |
| &nbsp;&nbsp;vmin_m_s | ≥ -0.05 | -0.12 | ❌ | -0.07 |
| &nbsp;&nbsp;rtf_mean | ≥ 0.9 | 0.85 | ❌ | -0.05 |
| &nbsp;&nbsp;path_efficiency | ≥ 0.55 | 0.4 | ❌ | -0.15 |
| &nbsp;&nbsp;completion_time_s | ≤ 60.0 | 85.0 | ❌ | +25.0 |
