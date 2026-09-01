# D1 算力维度判定表 (d1_judge.py)

| 问题 | 指标 | 阈值 | S1 | S2 |
|---|---|---|---|---|
| Q1 | FastLIO ave total 最大/趋势 (ms) | <70ms 且 后/前<1.3 | 38.0 / ×1.00 (2帧) [PASS] | 75.0 / ×1.00 (3帧) [FAIL] |
| Q1 | Open3D time_this_loc max (ms) | <100 | — [—] | 105.0 [FAIL] |
| Q1 | Nav2 超时/rate WARN 计数 (次) | =0 | 0 [PASS] | 1 [FAIL] |
| Q1 | /Odometry 平均频率 (Hz) | ≥9.5 | — [—] | — [—] |
| Q2 | PSI cpu some avg60 峰值 (%) | ≤5 | 2.2 [PASS] | 6.2 [FAIL] |
| Q2 | PSI mem some avg60 峰值 (%) | ≤0.1 | 0.00 [PASS] | 0.10 [PASS] |
| Q2 | vmstat si/so>0 行数 (行) | =0 | 0 [PASS] | 1 [FAIL] |
| Q2 | MemAvailable 最低 (MB) | >2048 | 9800 [PASS] | 1800 [FAIL] |
| Q2 | 峰值温度 (°C) | <90 | 60 [PASS] | 95 [FAIL] |
| Q2 | throttle_count 增量 (次) | =0 | 0 [PASS] | 3 [WARN] |
| Q2 | 隔离核 CPU4 %usr+%sys 平均 (%) | ≤2 | 0.08 [PASS] | 0.08 [PASS] |
| Q2 | 隔离核 CPU6 %usr+%sys 平均 (%) | ≤2 | 0.09 [PASS] | 0.09 [PASS] |
| Q2 | 最忙非隔离核峰值 (%) | ≤85(告警) | CPU10 62.1 [PASS] | CPU10 62.1 [PASS] |
| Q3 | cyclictest max 延迟 (µs) | <50(场景相关) | 42 [PASS] | 150 [FAIL] |
| Q3 | Control 唤醒延迟 p99 (max) (µs) | <200 | 45 [PASS] | 210 [FAIL] |
| Q3 | deadline miss (总/滑窗10min) (次) | 滑窗<10 | 1/1 [PASS] | 12/12 [FAIL] |
| Q3 | WKC 异常轮询合计 (次) | =0 | 0 [PASS] | 2 [FAIL] |
| Q3 | DC 偏差 max (µs) | <1.0(场景相关) | 0.85 [PASS] | 6.20 [FAIL] |
| Q3 | 总线 overrun (滑窗10min) (次) | <10 | 0 [PASS] | 12 [FAIL] |

- **S1 Q1**: PASS
- **S1 Q2**: PASS
- **S1 Q3**: PASS
- **S2 Q1**: FAIL
- **S2 Q2**: FAIL
- **S2 Q3**: FAIL
