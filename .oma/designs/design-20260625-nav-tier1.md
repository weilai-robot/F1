# Design: Tier 1 Navigation Fixes

> **Design ID:** design-20260625-nav-tier1
> **Status:** DRAFT
> **Scope:** Tier 1 items from NAV_IMPROVEMENT_ROADMAP.md
> **Goal:** 解决仿真中导航的 3 个致命问题: SLAM 漂移、速度跳变摔倒、无全局定位

---

## 问题定义

### 当前导航仿真中的致命问题

| # | 问题 | 现象 | 根因 |
|---|------|------|------|
| P0-1 | SLAM 漂移 | RViz 点云偏离静态地图, 走 ~5m 后偏移 0.3~0.8m | FastLIO2 纯增量 LIO 无全局校正 |
| P0-2 | 速度跳变摔倒 | Nav2 输出速度突变 → RL 步态紊乱 → 摔倒 | MPPI → RL 之间无加速度限制 |
| P0-3 | 点云偏移导致撞墙 | FastLIO2 body 系不准 → costmap 障碍物错位 | 同 P0-1, SLAM 漂移的下游影响 |

### 解决策略

仿真中我们有 MuJoCo ground truth (`/mujoco/ground_truth`)，可以**零成本替代 FastLIO2**。真机用 AMCL 做全局校正。速度跳变在 odom_bridge relay 链路中加 rate limiter。

---

## 方案设计

### Fix 1: 仿真 Ground Truth 定位模式 (T1.1)

**原理：** 仿真中 `/mujoco/ground_truth` 提供精确的 base pose (位置 + 姿态 + RTF + 碰撞 + 累计距离)，用它可以完全替代 FastLIO2 做定位。

**设计：** 在 `odom_bridge.py` 中新增 `ground_truth_mode` 参数，切换两种工作模式：

```
Mode A: "fastlio" (默认, 真机)
  订阅 /Odometry (FastLIO2) → 坐标转换 → odom→base_footprint TF + /odom

Mode B: "ground_truth" (仿真)
  订阅 /mujoco/ground_truth (Float64MultiArray) → 直接发布 → odom→base_footprint TF + /odom
  FastLIO2 仍运行 (供点云去畸变), 但定位不依赖它
```

**Float64MultiArray → Odometry 转换：**

```
/mujoco/ground_truth = [sim_t, x, y, z, roll, pitch, yaw, rtf, collisions, cum_dist]

转换:
  odom.pose.position = (x, y, 0)           # base_footprint Z=0
  odom.pose.orientation = yaw → quaternion  # 仅 yaw, pitch/roll 抹平
  odom.twist.linear.x = Δx / Δt             # 数值微分 (仿真中有 RTF, 不需要 FastLIO2 的 twist)
  odom.twist.angular.z = Δyaw / Δt

发布:
  TF: odom → base_footprint (translation=(x,y,0), rotation=yaw_only)
  Topic: /odom (含位姿 + 微分速度)
```

**优点：**
- 零漂移 (MuJoCo 精确位姿)
- 消除 FastLIO2 步态振动漂移
- RTF 计算已内建
- 点云去畸变仍由 FastLIO2 处理 (不影响感知质量)

**改动文件：** `navigation/humanoid_sim/scripts/odom_bridge.py` (~80 行新增)

---

### Fix 2: cmd_vel 加速度限制器 (T1.3)

**原理：** 在 Nav2 `/cmd_vel` → `/cmd_vel_limiter` 的 relay 链路中，对速度指令做加速度限幅，防止跳变冲击 RL 策略。

**设计：** 在 `odom_bridge.py` 的 `_cmd_vel_relay_cb` 中加 Rate Limiter：

```python
class VelocityRateLimiter:
    """限制速度变化率 (加速度), 平滑 Nav2 输出"""
    def __init__(self, max_ax=1.5, max_az=2.0, dt_min=0.01):
        self.max_ax = max_ax     # 线加速度上限 m/s²
        self.max_az = max_az     # 角加速度上限 rad/s²
        self.last_vx = 0.0
        self.last_wz = 0.0
        self.last_time = 0.0
        self.dt_min = dt_min

    def limit(self, vx, wz):
        now = time.monotonic()
        dt = max(now - self.last_time, self.dt_min)

        # 计算加速度并 clamp
        ax = (vx - self.last_vx) / dt
        az = (wz - self.last_wz) / dt
        ax = max(-self.max_ax, min(self.max_ax, ax))
        az = max(-self.max_az, min(self.max_az, az))

        # 积分回速度
        self.last_vx += ax * dt
        self.last_wz += az * dt
        self.last_time = now
        return self.last_vx, self.last_wz
```

**参数选择依据：**
- `max_ax = 1.5 m/s²`：人形步行安全加速度上限 (参考 ISO 13482)
  - 从 0→0.3m/s 需 0.2s (10 步 @ 50ms)，RL 策略可跟踪
- `max_az = 2.0 rad/s²`：转弯加速度上限
  - 从 0→0.4rad/s 需 0.2s，转弯不会失稳

**改动文件：** `navigation/humanoid_sim/scripts/odom_bridge.py` (~40 行新增)

---

### Fix 3: AMCL + pc2scan 启用 (T1.2, 真机用)

**原理：** 取消注释已有的 pc2scan launch + 启用 AMCL `tf_broadcast: True`。

**改动：**

1. `navigation.launch.py` — 取消注释 pc2scan launch
2. `nav2_mujoco.yaml` AMCL 配置：
   ```yaml
   amcl:
     ros__parameters:
       tf_broadcast: True
       set_initial_pose: true
       initial_pose: [0.0, 0.0, 0.0, 0.0]
   ```
3. `tf_bridge.launch.py` — 删除 `map→odom` 静态 TF (AMCL 会动态发布)

**改动文件：** 3 个配置文件 (~10 行修改)

---

## Ablation Plan

每项改进独立验证，用 `nav_test_runner.py` 跑定量对比：

| 实验 | SLAM 源 | 限速 | AMCL | 预期效果 |
|------|--------|------|------|---------|
| baseline | FastLIO2 | ❌ | ❌ | 当前状态 (漂移+摔倒) |
| +Fix2 | FastLIO2 | ✅ | ❌ | 不摔但会漂移撞墙 |
| +Fix1 | Ground Truth | ❌ | ❌ | 不漂但不摔 (速度仍跳变) |
| +Fix1+Fix2 | Ground Truth | ✅ | ❌ | 仿真目标: 不漂不摔不撞 |
| +Fix3 | Ground Truth | ✅ | ✅ | 真机目标 |

**场景矩阵 (6 个预定义场景):**

| 场景 | 目标 | 距离 | 关键挑战 |
|------|------|------|---------|
| A_straight_5m | (5, 0) | 5m | 基线直线 |
| B_obstacle_bypass | (5, 0) | 5m | 绕纸箱 |
| C_narrow_passage | (5, -3) | 7m | 穿通道A (0.8m) |
| D_impassable | (5, 3.2) | 7m | 绕通道B (0.35m) |
| E_long_distance | (8, -3) | 12m | 长距离漂移 |
| F_return_trip | (0, 0) | 10m | 往返 |

**通过标准:**
- 摔倒率 = 0%
- 碰撞率 < 10%
- 成功率 > 80%

---

## 文件改动清单

| 文件 | 改动类型 | 内容 |
|------|---------|------|
| `navigation/humanoid_sim/scripts/odom_bridge.py` | 改 | 新增 ground_truth_mode + VelocityRateLimiter |
| `navigation/humanoid_sim/launch/navigation.launch.py` | 改 | 取消 pc2scan 注释 |
| `navigation/humanoid_sim/config/nav2_mujoco.yaml` | 改 | AMCL 参数 + FastLIO2 配置备注 |
| `navigation/humanoid_sim/launch/tf_bridge.launch.py` | 改 | 删除 map→odom 静态 TF (真机) |
| `run_sim_nav.sh` | 改 | 传递 ground_truth_mode 参数给 odom_bridge |

---

## 依赖与前置条件

- [x] sim_module 已发布 `/mujoco/ground_truth` (Float64MultiArray)
- [x] odom_bridge.py 已有 cmd_vel relay 逻辑
- [x] pc2scan.launch.py 已存在 (被注释)
- [x] nav_test_runner.py 已有 6 个场景定义
- [ ] drift_check.py 验证当前漂移基线 (需要跑一次)
