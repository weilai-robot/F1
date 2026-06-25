# F1 Navigation Improvement Roadmap

> **基于 2024-2026 人形机器人导航 SOTA 调研，结合 F1 当前系统的实际瓶颈制定。**
> **调研来源：** P3O-CBF (arXiv:2508.07611), Omni-Perception (arXiv:2505.19214),
> Cerberus2.0 (ICRA 2023), elevation_mapping_cupy (ETH), Agile-but-Safe (RSS 2024),
> Super-LIO (arXiv:2509.05748), Okawara Tightly-Coupled LILO (RAL 2025)

---

## 当前系统的 4 个核心瓶颈

| 瓶颈 | 根因 | 实际表现 | 影响 |
|------|------|---------|------|
| **SLAM 漂移** | FastLIO2 纯增量 LIO，无腿运动学前馈，无回环检测 | RViz 中点云逐渐偏离静态地图 | 长距离导航失败 |
| **2D costmap 盲区** | VoxelLayer 3D→2D 压扁丢失高度信息 | 看不到悬挂障碍，桌子下方误判为障碍 | 撞墙/绕路 |
| **MPPI 轮式假设** | DiffDrive 运动模型不理解双足步态 | 原地旋转导致 RL 策略步态紊乱 | 摔倒 |
| **膨胀层叠** | 静态地图 + global inflation + local inflation 三重叠加 | 0.8m 通道被完全堵死 | 规划失败 |

---

## Tier 1: 近期优化 (1-2 周，不改架构)

> 在 Nav2 + FastLIO2 框架内解决最痛的问题。

### T1.1 仿真：Ground Truth 替代 FastLIO2 定位

| | 当前 | 改进后 |
|--|------|--------|
| odom 来源 | FastLIO2 LIO（漂移） | `/mujoco/ground_truth`（零漂移） |
| 修改 | odom_bridge.py 订阅 /Odometry | odom_bridge.py 订阅 /mujoco/ground_truth |
| 效果 | 消除仿真中所有 SLAM 漂移问题 | RViz 点云完美贴合静态地图 |

**工作量：** 修改 `odom_bridge.py` 订阅源 + TF 发布逻辑（~50 行）

### T1.2 真机：启用 AMCL + pc2scan

| | 当前 | 改进后 |
|--|------|--------|
| 全局定位 | 无（map→odom 静态 TF） | AMCL 动态校正 |
| /scan 来源 | 无 | pointcloud_to_laserscan 切割 3D 点云 |
| 漂移修正 | 不可校正 | AMCL 在地图中做绝对定位闭环 |

**工作量：**
1. 取消注释 `pc2scan.launch.py`（已存在）
2. AMCL `tf_broadcast: True`, `set_initial_pose: true`
3. 删除 `tf_bridge.launch.py` 中的 `map→odom` 静态 TF

**参考：** Nav2 标准 AMCL 配置

### T1.3 加速度限制器

| | 当前 | 改进后 |
|--|------|--------|
| Nav2→RL 速度链路 | MPPI 直接输出 → /cmd_vel_limiter → RL | MPPI → rate_limiter → /cmd_vel_limiter → RL |
| 问题 | MPPI 速度跳变导致 RL 步态紊乱 | 平滑过渡 |

**方案：** 在 odom_bridge.py 的 cmd_vel relay 中加加速度限制
```
vx_rate_limit: 1.5 m/s²  (人形安全上限)
wz_rate_limit: 2.0 rad/s²
```

**工作量：** ~30 行 Python（在 relay callback 中加 rate limiter）

### T1.4 腿运动学前馈 (SLAM 漂移缓解)

| | 当前 | 改进后 |
|--|------|--------|
| FastLIO2 运动先验 | 仅 IMU 预积分 | IMU + leg odometry |
| 漂移率 | ~5% 行走距离 | <1% 行走距离 |

**参考方案：** Cerberus2.0 (开源 https://github.com/ShuoYangRobotics/Cerberus2.0)
- 视觉+惯性+腿运动学融合 (VILO)
- 在线运动学参数标定（处理连杆变形/加工误差）
- 接触离群点剔除（处理脚滑）

**工作量：** 集成 Cerberus2.0 或在 FastLIO2 中加 leg odometry 因子（~3 天）

### Tier 1 完成后的预期效果

```
仿真:  ground truth 替代 → 零漂移 → 稳定导航不摔不撞
真机:  AMCL 校正 + 加速度限制 → 可用的基础导航
指标:  成功率 > 80%, 摔倒率 < 5% (短距离 < 5m)
```

---

## Tier 2: 中期升级 (1-2 月，替换关键组件)

> 替换不适合人形机器人的组件，进入 3D 感知。

### T2.1 elevation_mapping_cupy (GPU 2.5D 高程图)

**替换：** Nav2 VoxelLayer (2D 压扁) → elevation_mapping_cupy (2.5D 高程)

| | 2D VoxelLayer | elevation_mapping_cupy |
|--|--------------|----------------------|
| 表示 | 二值占据 (有/无障碍) | 每格高度 + 方差 + 可通行性 |
| 悬挂障碍 | ❌ 压扁丢失 | ✅ Visibility Cleanup 正确识别 |
| 地面可通行性 | ❌ 只有障碍/自由 | ✅ 学习坡度/粗糙度 |
| 高度漂移补偿 | ❌ 无 | ✅ 内建 Height Drift Compensation |
| 计算平台 | CPU | GPU (CuPy) |

**开源：** https://github.com/leggedrobotics/elevation_mapping_cupy (983★)

**工作量：** ROS2 节点集成 + GPU 驱动 + Nav2 costmap adapter（~1 周）

### T2.2 Super-LIO 替代 FastLIO2

| | FastLIO2 | Super-LIO |
|--|---------|----------|
| ARM 每帧耗时 | ~80ms | ~17ms (4.7× 加速) |
| X86 每帧耗时 | ~19ms | ~4.5ms (4.2× 加速) |
| 精度 (RMSE) | 0.81m | 0.74m |
| Livox Mid-360 | ✅ | ✅ |

**开源：** https://github.com/Liansheng-Wang/Super-LIO

**工作量：** 替换 colcon 包 + 配置文件（~2 天）

### T2.3 Smac Hybrid-A* 替代 NavfnPlanner

| | NavfnPlanner (Dijkstra) | SmacPlannerHybrid (A*) |
|--|------------------------|----------------------|
| 搜索方式 | 无方向 4/8 连通 | 角度感知 (x,y,θ) 搜索 |
| 狭窄通道 | 差（经常绕远路） | 好（直接穿过） |
| U 形障碍 | 易卡死 | 不卡 |
| 非完整约束 | ❌ | ✅ 支持最小转弯半径 |

**工作量：** 改 nav2_mujoco.yaml planner 配置（~1 天）

### T2.4 RPA-MPPI (排斥势场增强)

**参考：** arXiv:2410.11379 (2025)

在 MPPI 的 cost function 中加入排斥势场项，消除局部最小值：

```
conventional_cost = path_align + obstacle + goal
RPA_cost = conventional + repulsive_potential(obstacle)
```

**工作量：** 实现 MPPI Critic 扩展（~3 天）

### Tier 2 完成后的预期效果

```
室内导航: 3D 感知 + 高效 SLAM + 角度感知规划
指标: 成功率 > 90%, 可处理悬挂障碍 + 狭窄通道 + 长距离 (15m+)
真机可用: Super-LIO 省 75% CPU 留给感知
```

---

## Tier 3: 长期方向 (3-6 月，架构跃迁)

> 跳出 Nav2 框架，进入端到端学习。

### T3.1 P3O-CBF: 端到端 RL 导航 (LiDAR → cmd_vel)

**参考：** arXiv:2508.07611 (HKUST-GZ / IIT, 2025)

| | Nav2 + MPPI | P3O-CBF |
|--|------------|---------|
| 架构 | SLAM→Map→Plan→Control (4级) | LiDAR→GRU→MLP→cmd_vel (1级) |
| 感知 | 2D costmap (压扁) | 原始 3D 点云 (完整) |
| 安全 | reward shaping (启发式) | CBF 约束 (形式化保证) |
| 悬挂障碍 | ❌ | ✅ 83% 成功率 |
| 动态障碍 | 56% (PPO) | 86% (P3O-CBF) |
| 延迟 | ~200ms (全链路) | ~5ms (单次推理) |

**架构：**
```
Livox Mid-360 原始点云
  ↓ 点云编码器 → 64-dim 向量
关节本体感知 (10 步历史)
  ↓ GRU (时序融合)
  ↓ MLP [1024, 512, 256, 128]
  ↓ 12 关节位置目标
```

**安全机制：**
- CMDP (Constrained MDP): 安全作为 cost 而非 reward
- CBF (Control Barrier Function): h_D = 签名距离 − D_min
- P3O 优化: Penalized Proximal Policy Optimization

**训练：**
- Isaac Sim / Genesis (sim2sim)
- MuJoCo-LiDAR 仿真工具包 (已有)
- Domain randomization + curriculum

**开源：** https://github.com/aCodeDog/SafeHumanoidsPolicy

**工作量：**
- 训练框架改造: ~2 周 (复用 agibot_x1_train)
- 奖励/CBF 设计: ~2 周
- 训练调优: ~4 周
- Sim2real: ~4 周

### T3.2 Agile-but-Safe: 安全网策略

**参考：** arXiv:2401.17583 (RSS 2024)

三组件架构：
1. **敏捷策略**: 高速导航 (>1.0 m/s)
2. **恢复策略**: 检测到失稳时接管，防摔
3. **Reach-avoid 网络**: 学习何时切换

**开源：** https://github.com/LeCAR-Lab/ABS

**工作量：** 与 T3.1 联合训练（~1 周 incremental）

### Tier 3 完成后的预期效果

```
全自主导航: 无 SLAM / 无 costmap / 无路径规划器
点云 → 关节指令 直通，延迟 <10ms
动态障碍 86% 成功率，悬挂障碍 83% 成功率
```

---

## 实施优先级矩阵

```
                    高收益
                       │
    T2.1               │               T3.1
  elevation_map        │            P3O-CBF
  (3D感知)              │          (端到端RL)
                       │
 ──────────────────────┼──────────────────────
  低投入                │               高投入
                       │
    T1.1   T1.3  T1.2  │    T2.3  T2.2
  GT替代  限速  AMCL   │  SmacA* SuperLIO
                       │
                    低收益
```

**推荐执行顺序：**
1. T1.1 (ground truth 替代) → 立即解决仿真漂移
2. T1.3 (加速度限制) → 立即解决摔倒
3. T1.2 (AMCL) → 解决真机定位
4. T2.1 (elevation_mapping) → 解决 3D 避障
5. T2.3 (Smac A*) → 解决通道问题
6. T3.1 (P3O-CBF) → 长期方向

---

## F1 项目特殊优势

1. **已有 RL locomotion 基础设施**：ONNX 推理 + Isaac Gym 训练 + MuJoCo-LiDAR
   → T3.1 端到端导航 RL 的训练成本远低于从零开始

2. **已有 MuJoCo ground truth**：sim_module 精确知道机器人位姿
   → T1.1 零成本实现，且为 T3.1 训练提供完美标签

3. **Livox Mid-360**：非重复扫描 + 360° 覆盖
   → P3O-CBF 和 Omni-Perception 均验证过此雷达

4. **已有测试框架**：nav_test_runner.py + drift_check.py
   → 每个改进项都有量化对比基准

---

## 各 Tier 对照表

| 维度 | Tier 1 (近期) | Tier 2 (中期) | Tier 3 (长期) |
|------|-------------|-------------|-------------|
| 架构 | Nav2 + FastLIO2 | Nav2 + 3D 感知 | 端到端 RL |
| SLAM | GT (仿真) / AMCL (真机) | Super-LIO + leg odom | 不需要 SLAM |
| Costmap | 2D VoxelLayer | elevation_mapping_cupy | 不需要 costmap |
| 规划器 | NavfnPlanner | Smac Hybrid-A* | 不需要规划器 |
| 控制器 | MPPI (加速度限制) | RPA-MPPI | RL policy |
| 3D 感知 | ❌ | ✅ (2.5D 高程图) | ✅ (原始点云) |
| 安全 | 启发式 | 启发式 + 势场 | CBF 形式化保证 |
| 工作量 | 1-2 周 | 1-2 月 | 3-6 月 |
| 成熟度 | 生产可用 | 生产可用 | 研究前沿 |
