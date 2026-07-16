# Agent 导航算法迭代操作手册

> 本文档供 AI Agent 自动化迭代导航算法使用。描述完整的 push → CI → 结果获取 → 分析 闭环。

## 流水线架构

```
Agent 修改 navigation 代码
    │
    ├── 1. git push origin nav/dev-<描述>
    │
    ▼
GitHub Actions 自动触发 (nav_eval.yml)
    │
    ├── Runner 自动执行:
    │   ├── checkout (增量, 不清理 build/)
    │   ├── build motion_control (增量)
    │   ├── build navigation (增量)
    │   ├── ci_nav_test.sh (无人值守仿真测试)
    │   │   ├── Xvfb 无头渲染
    │   │   ├── 启动全链路 (aimrt + lidar + fastlio + open3d + nav2)
    │   │   ├── 切 walk_mode
    │   │   ├── nav_test_runner.py (跑场景 + 采集指标)
    │   │   └── 组件日志诊断
    │   └── 上传 artifact (reports/*.json)
    │
    ▼
Agent 获取结果
    │
    ├── 方式 A: GitHub Webhook (事件驱动, 推荐)
    │   CI 完成 → GitHub 自动 POST → webhook_receiver.py → 下载 + 分析
    │
    ├── 方式 B: gh CLI 轮询 (同步阻塞)
    │   python3 scripts/fetch_ci_results.py            # 阻塞等待 + 自动分析
    │
    └── 方式 C: GitHub Actions 页面 (人工查看)
        Step Summary 直接显示指标表 + 组件诊断
    │
    ▼
Agent 分析 JSON → 决定下一次修改 → 回到步骤 1
```

## 方式 A: Webhook 事件驱动 (推荐)

CI 跑完后 GitHub 自动通知 Agent，无需轮询。

### 1. 在 Agent 机器上启动 Webhook Receiver

```bash
# 设置 webhook 密钥 (自定义, 与 GitHub 配置一致)
export WEBHOOK_SECRET=my_secret_123

# 启动 (常驻)
python3 scripts/webhook_receiver.py --port 9090 --secret my_secret_123

# 可选: 指定收到结果后的回调命令
python3 scripts/webhook_receiver.py \
    --callback 'python3 my_agent.py --ci-report $CI_REPORT_FILE'
```

### 2. 在 GitHub 配置 Webhook

```
GitHub → F1 仓库 → Settings → Webhooks → Add webhook

  URL:           http://<agent-public-ip>:9090/webhook
  Content type:  application/json
  Secret:        my_secret_123
  Events:        ✓ Let me select individual events → ✓ Workflow runs
```

### 3. 事件流

```
CI 完成 → GitHub 发 workflow_run (action=completed) webhook
    → webhook_receiver.py 收到, 验证签名
    → 自动执行 gh run download 下载 artifact
    → 自动执行 fetch_ci_results.py 分析结果
    → 输出 ci_fetched/report.md
    → 触发 --callback 指定的回调命令
```

回调环境变量: `CI_RUN_ID`, `CI_RUN_NUMBER`, `CI_BRANCH`, `CI_CONCLUSION`,
`CI_REPORT_DIR`, `CI_REPORT_FILE`

## 关键文件

| 文件 | 职责 | 部署位置 |
|------|------|---------|
| `scripts/ci_nav_test.sh` | 无人值守仿真测试 (启停全链路 + 健康检查) | Runner 机器 |
| `scripts/nav_test_runner.py` | 导航场景执行 + 指标计算 + 诊断数据采集 | Runner 机器 |
| `.github/workflows/nav_eval.yml` | GitHub Actions 工作流定义 | GitHub 仓库 |
| `scripts/fetch_ci_results.py` | 结果下载 + 分析工具 | Agent 机器 |
| `scripts/webhook_receiver.py` | Webhook 事件接收器 (事件驱动模式) | Agent 机器 |

## 结果 JSON 结构

```json
{
  "scenario": "A_straight_5m",
  "metrics": {
    "success": false,
    "fall": true,
    "collisions": 2,
    "position_error_m": 3.765,
    "drift_mean_m": 252.3,
    "drift_max_m": 826.0,
    "linear_jerk_rms": 9352.0,
    "result_status": "FAILED_6"
  },
  "diagnostics": {
    "trajectory_gt": [[t, x, y, z, roll, pitch, yaw], ...],
    "drift_curve": [[t, drift_m], ...],
    "cmd_vel_curve": [[t, vx, wz], ...],
    "velocity_curve": [[t, v], ...],
    "events": [{"t": 0.0, "type": "test_start"}, ...],
    "fall_analysis": {"fall_time_s": 15.3, "fall_pitch_deg": 48.0, ...}
  }
}
```

另有 `ci_diagnostic.json` 包含各组件日志中的 error/warning。

## Agent 迭代循环

```bash
# === 迭代 N ===

# 1. 基于上一次结果修改代码
vim navigation/...

# 2. 推送到 nav/dev-* 分支
git checkout -b nav/dev-iteration-${N}
git add -A
git commit -m "nav: <改动描述>"
git push origin nav/dev-iteration-${N}

# 3. 等待 CI + 获取结果
python3 scripts/fetch_ci_results.py --branch nav/dev-iteration-${N}

# 4. 读取诊断报告
cat ci_fetched/report.md

# 5. 分析, 决定下一步
#    - 摔倒? → 看 fall_analysis, 调步态/控制参数
#    - 漂移? → 看 drift_curve 趋势, 调 FastLIO2 参数
#    - Nav2 error? → 看 ci_diagnostic.json 的具体错误
#    - 成功? → 跑 batch 全量场景验证

# 回到 1, 进入迭代 N+1
```

## 测试场景

| 场景 | 目标 | 超时 | 说明 |
|------|------|------|------|
| `A_straight_5m` | (5,0) | 60s | 直线基线 |
| `B_obstacle_bypass` | (5,0) | 60s | 绕障碍物 |
| `C_narrow_passage` | (5,-3) | 90s | 穿狭窄通道 |
| `D_impassable` | (5,3.2) | 90s | 不可通过 (应绕路) |
| `E_long_distance` | (8,-3) | 120s | 长距离对角 |
| `F_return_trip` | (0,0) | 120s | 往返 |

## 分支约定

| 分支 | 用途 |
|------|------|
| `main` | 稳定版, 不触发 CI |
| `nav/dev-*` | 开发分支, push 自动触发 CI |
| `nav/dev-iteration-N` | 第 N 轮迭代的特定改动 |
