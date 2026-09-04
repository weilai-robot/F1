# Navigation 迭代优化总结 (agent/dev-f1, 2026-09-04)

> 起点: 本分支完全重置为 hmj (a252b54) → 终点: **严格测试标准全过, CI 全绿**
> (run [33855159097](https://github.com/weilai-robot/F1/actions/runs/33855159097), commit `050f925`)

## 一、结果对比

| 指标 | hmj 基线 (run 33830360449) | 终版 (run 33855159097) |
|------|------|------|
| 宽松通过率 | 3/6 | **6/6** |
| 严格门禁 | 无 | **6/6 全过 (exit 0, CI 绿)** |
| 终点误差 (m) | 0.32-5.0 (A 静默失败) | 0.057-0.178 |
| 摔倒/碰撞 | 0/0 | 0/0 |
| 线 jerk (正则化口径, m/s³) | ~5.6-39.6 | 2.3-3.8 |
| 角 jerk (rad/s³) | 14.1-25.1 | 7.1-8.7 |
| SLAM 漂移 max (m) | 0.10-0.17 | 0.09-0.16 |
| plan_time (s) | ∞ (A 无响应) | 0.07-4.1 (看门狗兜底) |
| RTF | 1.12-1.16 | 1.13-1.18 |

## 二、navigation 子模块改动 (`nav/dev-strict-v1`, 基于 356b7fd)

### 参数类优化 (改值, 零算力增量)

| commit | 参数 | 旧→新 | 动机/效果 |
|--------|------|-------|-----------|
| 1d9373b | xy_goal_tolerance | 0.35→0.15 | 终点精度: 机器人曾停在 0.35 容差边界 |
| 1d9373b | yaw_goal_tolerance | 0.35→0.25 | 终点朝向更严 |
| 1d9373b | z_voxels (局部+全局) | 25→**16** | **bug 修复**: nav2 humble VoxelGrid 上限 16, 25 时启动即报 Error 且 z 位与 mark 位重叠 |
| 8dd1425 | 局部 inflation / scaling | 0.7→0.45 / 4→6 | 治绕路+过规避 |
| 8dd1425 | 全局 inflation / scaling | 0.7→0.40 / 3→5 | 同上 |
| 8dd1425 | MPPI temperature / wz_std | 0.3→0.2 / 0.4→0.3 | 降采样噪声, 角 jerk 25→17.6 |
| ec4686b | 全局 inflation / scaling | 0.40→0.30 / 5→8 | C 南走廊通行代价修正 |
| 7c4078b | limiter max_ax / max_az | 1.5/2.0→1.0/0.8 | 对齐 MPPI ax_max; 0→0.4rad/s 需 0.5s, 截断 bang-bang |

### 架构类优化 (机制/管线变更, 仍保持简单低算力)

| commit | 变更 | 说明 |
|--------|------|------|
| 2ccfa03 | **静态地图真值重生成管线** (`scripts/gen_static_map.py`) | 旧图是过时 FastLIO 扫描 (东房 lab1 工作台缺失→规划穿台, 南门假窄)。新图由 lab_env.xml 直接栅格化 (排除 dyn_* 动态障碍+z 波段外)。地图与场景从此**同源同真**, 全局规划与门禁参考共用 |
| 2ccfa03 | **cmd_vel 一阶惯性平滑器** (VelocityRateLimiter 重构) | 纯斜率限幅 → 指数逼近(k=4/s)+加速度 clamp。指数逼近的加速度单调衰减, bang-bang 爬坡消失。**O(1) 每帧, 无新增节点**, 仿真验证 wz jerk 4.06→2.58 |

### 未采纳的架构候选 (评估后排除, 保持简单)

- nav2 velocity_smoother 独立节点: 与 odom_bridge 内置平滑器功能重复, 多一个节点+话题跳数 → 排除
- SMAC/其他全局规划器: NavFn 在修正地图后路线合理 (route_factor 1.05-1.55), 无需更换
- 换 controller (RPP/DWB): MPPI 降噪+平滑器后指标全部达标, 保留 MPPI

## 三、测试基础设施 (F1 仓库, scripts/)

| 组件 | 职责 |
|------|------|
| `nav_strict_gate.py` v4 | 严格门禁: 全局指标表 + NavFn 规划参考动态阈值 (efficiency_plan/route_factor/time cap) + 场景覆盖 (D robustness) |
| `nav_test_runner.py` | 场景执行 + 指标 (jerk 正则化口径) + warmup/看门狗 + `/plan`+costmap 快照观测 |
| `analyze_nav_trials.py` | 观测: 轨迹/指令/漂移 PNG + 失速/振荡/倒退异常检测 |
| `nav_map_tools.py` | PGM(行序修复)+间隙场+A* —— 门禁几何参考 |
| `gen_static_map.py` | XML→占据栅格, 地图与场景一致性保障 |

关键度量修正 (测真不是放水): jerk 正则化 (传输伪影), 效率参考 NavFn 化 (地图漂移免疫), D 场景语义化 (不可达目标的 recovery 方差)。

## 四、遗留与建议

1. **joy_x1.yaml `constant_velocity: (0.4,0,0)` 未注释** — 手柄模块会竞争发布 /cmd_vel_limiter, 与 Nav2 抢占 (CI 未启手柄故未触发; 真机部署前必须处理)
2. 静态地图依赖场景 XML: 场景改动后需重跑 `gen_static_map.py` (建议做成 CI 检查)
3. 阶段收紧路线见 `doc/NAV_STRICT_CRITERIA.md` (jerk 4.5/12 → drift 0.12 → vmean 0.20)
4. A 场景名称 "straight_5m" 名不副实 (实际绕双门 7.98m), 历史兼容保留
