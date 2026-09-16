#!/bin/bash
# ============================================================
#  nav_debug_record.sh — 真机导航"同一世界系"调试日志抓取
#
#  一次采集三路核心数据 (后续由 nav_debug_plot.py 统一到 map 系对齐):
#    1. 全局路径          /plan                        (map 系)
#    2. MPPI 最优局部轨迹 /nav_debug/mppi_optimal_traj (odom 系, 经 map->odom TF 对齐)
#    3. 限速后速度指令    /cmd_vel_limiter             (对照 /cmd_vel 原始输出)
#
#  辅助数据 (用于坐标对齐与现场解读):
#    /odom、/tf、/tf_static、/transformed_global_plan(MPPI 跟踪参考段)、
#    导航 action 状态与反馈、/local_costmap/costmap(可关)
#
#  用法:
#    ./nav_debug_record.sh                      # 录到 Ctrl+C
#    ./nav_debug_record.sh -d 120               # 录 120 秒
#    ./nav_debug_record.sh -d 120 walk_stuck    # 带标签
#    ./nav_debug_record.sh --no-costmap         # 不录局部代价地图
#
#  输出:
#    F1/reports/nav_debug/<时间戳>[_标签]/
#      notes.md            — 测试元数据模板（请填写现象）
#      pre_snapshot/       — 开录前一次性快照
#      post_snapshot/      — 收尾快照 (bag info 等)
#      bag/nav_debug/      — rosbag2
#      relay.log           — /trajectories -> 最优轨迹中继日志
#
#  建议流程:
#    1. run_nav_real.sh 栈已起, 小脑 AimRT 已启动, 机器人已 stand
#    2. 本脚本开始采集
#    3. ./switch_x1_mode.sh walk  →  ./send_nav_goal_real.sh <x> <y>
#    4. 出现"原地踏步"等异常后停留几秒再 Ctrl+C（或用 -d 定时）
#    5. python3 scripts/nav_debug_plot.py reports/nav_debug/<目录>
#       → 生成 map 系复盘图 + CSV + 自动摘要
#
#  注意:
#    挂载中继后 MPPI 每周期会额外生成可视化 marker (controller 负载略增),
#    仅建议调试期间使用; 本脚本退出时会自动清理 relay。
# ============================================================
set -u

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
NAV_DIR="${ROOT_DIR}/navigation"
REPORT_ROOT="${ROOT_DIR}/reports/nav_debug"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

DURATION=0          # 0 = 录到 Ctrl+C
LABEL=""
WITH_COSTMAP=true

usage() {
  sed -n '2,39p' "$0"
  exit "${1:-0}"
}

while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help) usage 0 ;;
    -d|--duration)
      DURATION="${2:?missing duration}"
      shift 2
      ;;
    --no-costmap) WITH_COSTMAP=false; shift ;;
    --*)
      echo -e "${RED}[ERROR] 未知选项: $1${NC}" >&2
      usage 1
      ;;
    *)
      LABEL="$1"
      shift
      ;;
  esac
done

STAMP="$(date +%Y%m%d_%H%M%S)"
SAFE_LABEL="${LABEL//[^a-zA-Z0-9_-]/_}"
if [ -n "${SAFE_LABEL}" ]; then
  RUN_DIR="${REPORT_ROOT}/${STAMP}_${SAFE_LABEL}"
else
  RUN_DIR="${REPORT_ROOT}/${STAMP}"
fi
SNAP_DIR="${RUN_DIR}/pre_snapshot"
POST_DIR="${RUN_DIR}/post_snapshot"
BAG_DIR="${RUN_DIR}/bag"
BAG_NAME="${BAG_DIR}/nav_debug"

# ---- 话题清单 ----
# 核心三路 + 对齐/解读辅助; /trajectories 经中继转成紧凑 Path 后记录
# 定位质量链: /odom2map + /localization_3d_confidence (ICP 矩阵/fitness, 2Hz)
# 原始链: /Odometry (FastLIO 输出, 区分 LIO 与 bridge 层问题)
# 模式话题: 状态机切换证据 (排除"假卡停": stand/keep 抢占)
TOPICS=(
  /tf
  /tf_static
  /plan
  /nav_debug/mppi_optimal_traj
  /transformed_global_plan
  /cmd_vel
  /cmd_vel_limiter
  /odom
  /Odometry
  /odom2map
  /localization_3d_confidence
  /walk_mode
  /stand_mode
  /zero_mode
  /idle_mode
  /keep_mode
  /navigate_to_pose/_action/status
  /navigate_to_pose/_action/feedback
)
if [ "${WITH_COSTMAP}" = true ]; then
  TOPICS+=(/local_costmap/costmap)
fi

# /nav_debug/mppi_optimal_traj 由本脚本拉起的 relay 发布, 不参与"开录前存在性检查"
PRECHECK_TOPICS=(/tf /tf_static /plan /cmd_vel /cmd_vel_limiter /odom /Odometry)

mkdir -p "${SNAP_DIR}" "${POST_DIR}" "${BAG_DIR}"

echo -e "${GREEN}[nav_debug] 输出目录:${NC} ${RUN_DIR}"

# ---- ROS2 环境 ----
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/ros2_source.sh"
if [ -f "${NAV_DIR}/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  source "${NAV_DIR}/install/setup.bash"
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo -e "${RED}[ERROR] ros2 未找到，请先 source ROS2 环境${NC}" >&2
  exit 1
fi

if [ ! -f "${SCRIPT_DIR}/nav_debug_logger.py" ]; then
  echo -e "${RED}[ERROR] 缺少 ${SCRIPT_DIR}/nav_debug_logger.py${NC}" >&2
  exit 1
fi

snapshot() {
  local name="$1"
  shift
  local outfile="${SNAP_DIR}/${name}.txt"
  {
    echo "# command: $*"
    echo "# time: $(date --iso-8601=seconds)"
    echo
    "$@"
  } >"${outfile}" 2>&1 || true
}

snapshot_shell() {
  local name="$1"
  local cmd="$2"
  local outfile="${SNAP_DIR}/${name}.txt"
  {
    echo "# command: ${cmd}"
    echo "# time: $(date --iso-8601=seconds)"
    echo
    bash -lc "${cmd}"
  } >"${outfile}" 2>&1 || true
}

post_snapshot_shell() {
  local name="$1"
  local cmd="$2"
  local outfile="${POST_DIR}/${name}.txt"
  {
    echo "# command: ${cmd}"
    echo "# time: $(date --iso-8601=seconds)"
    echo
    bash -lc "${cmd}"
  } >"${outfile}" 2>&1 || true
}

# ---- 开录前检查 ----
echo -e "${YELLOW}[nav_debug] 检查关键话题...${NC}"
MISSING=()
TOPIC_LIST="$(ros2 topic list 2>/dev/null || true)"
for t in "${PRECHECK_TOPICS[@]}"; do
  if ! grep -qx "${t}" <<<"${TOPIC_LIST}"; then
    MISSING+=("${t}")
  fi
done
if [ ${#MISSING[@]} -gt 0 ]; then
  echo -e "${RED}[WARN] 以下话题当前不存在（请确认导航栈已启动）:${NC}" >&2
  printf '  %s\n' "${MISSING[@]}" >&2
fi

if ! grep -qx "/trajectories" <<<"${TOPIC_LIST}"; then
  echo -e "${YELLOW}[WARN] /trajectories 不存在 — MPPI 未激活或 visualize 关闭,${NC}" >&2
  echo -e "${YELLOW}       最优局部轨迹将缺失（其余数据不受影响）${NC}" >&2
fi

echo -e "${YELLOW}[nav_debug] 开录前快照...${NC}"
snapshot "topic_list" ros2 topic list -t
snapshot "node_list" ros2 node list
snapshot_shell "lifecycle_controller_server" "timeout 5s ros2 lifecycle get /controller_server"
snapshot_shell "mppi_visualize" "timeout 5s ros2 param get /controller_server FollowPath.visualize"
snapshot_shell "tf_map_odom" "timeout 3s ros2 run tf2_ros tf2_echo map odom"
snapshot_shell "tf_odom_footprint" "timeout 3s ros2 run tf2_ros tf2_echo odom base_footprint"
# 配置/代码溯源 (mount_rpy 影响 /odom 朝向与 twist 坐标系; max_ax/az 影响 /cmd_vel_limiter;
# MPPI critic 权重影响两轮数据的可比性) — 保证事后能把录包数据与当时的代码版本/参数对应起来
snapshot_shell "odom_bridge_params" "timeout 8s ros2 param dump /odom_bridge"
snapshot_shell "controller_server_params" "timeout 8s ros2 param dump /controller_server"
snapshot_shell "nav_code_version" "timeout 5s ros2 pkg prefix humanoid_sim; git -C ${ROOT_DIR}/navigation rev-parse HEAD; git -C ${ROOT_DIR}/navigation branch --show-current; git -C ${ROOT_DIR}/navigation status --short"
snapshot_shell "motion_control_version" "git -C ${ROOT_DIR}/motion_control rev-parse HEAD; git -C ${ROOT_DIR}/motion_control branch --show-current; git -C ${ROOT_DIR}/motion_control status --short | head -20"
# 小脑生效配置快照 (cmd_threshold/sw_mode 等决定死区行为; 依次找源/构建/安装三处)
for _rl_cfg in "${ROOT_DIR}/build/cfg/control_module/rl_x1.yaml" \
               "${ROOT_DIR}/motion_control/install/linux/bin/cfg/control_module/rl_x1.yaml" \
               "${ROOT_DIR}/motion_control/module/control_module/cfg/rl_x1.yaml"; do
  if [ -f "${_rl_cfg}" ]; then
    cp "${_rl_cfg}" "${SNAP_DIR}/rl_x1.yaml"
    echo "# source: ${_rl_cfg}" >"${SNAP_DIR}/rl_x1.source.txt"
    break
  fi
done

# ---- 启动最优轨迹中继 (/trajectories -> /nav_debug/mppi_optimal_traj) ----
RELAY_PID=""
REC_PID=""
cleanup() {
  # 先停录包 (SIGINT 让 rosbag2 正常写 metadata), 再停 relay
  if [ -n "${REC_PID}" ] && kill -0 "${REC_PID}" 2>/dev/null; then
    kill -INT "${REC_PID}" 2>/dev/null || true
    wait "${REC_PID}" 2>/dev/null || true
  fi
  if [ -n "${RELAY_PID}" ] && kill -0 "${RELAY_PID}" 2>/dev/null; then
    kill -INT "${RELAY_PID}" 2>/dev/null || true
    wait "${RELAY_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT
# Ctrl+C 只置标记不立即退出, 保证后续收尾快照 / notes.md 仍会执行
INT_RECEIVED=0
trap 'INT_RECEIVED=1' INT TERM

echo -e "${GREEN}[nav_debug] 启动最优轨迹中继 (relay.log)...${NC}"
python3 "${SCRIPT_DIR}/nav_debug_logger.py" >"${RUN_DIR}/relay.log" 2>&1 &
RELAY_PID=$!
sleep 2

TOPIC_LIST="$(ros2 topic list 2>/dev/null || true)"
if grep -qx "/nav_debug/mppi_optimal_traj" <<<"${TOPIC_LIST}"; then
  echo -e "${GREEN}  [relay] /nav_debug/mppi_optimal_traj 已发布${NC}"
else
  echo -e "${RED}[WARN] relay 未就绪, 最优轨迹可能缺失。查看 relay.log${NC}" >&2
fi

# ---- 录包 ----
echo -e "${YELLOW}[nav_debug] topics (${#TOPICS[@]}):${NC}"
printf '  %s\n' "${TOPICS[@]}"

RECORD_CMD=(ros2 bag record -o "${BAG_NAME}" "${TOPICS[@]}")

rc=0
echo -e "${GREEN}[nav_debug] 开始录包 → ${BAG_NAME}${NC}"
if [ "${DURATION}" -gt 0 ]; then
  echo -e "${YELLOW}  时长 ${DURATION}s，到时自动停止${NC}"
  timeout --signal=INT "${DURATION}" "${RECORD_CMD[@]}" &
  REC_PID=$!
  wait "${REC_PID}"
  rc=$?
  REC_PID=""
  if [ "${rc}" -eq 124 ]; then
    echo -e "${GREEN}[nav_debug] 已达 ${DURATION}s，录包结束${NC}"
    rc=0
  fi
else
  echo -e "${YELLOW}  Ctrl+C 停止（停止后会自动收尾）${NC}"
  "${RECORD_CMD[@]}" &
  REC_PID=$!
  wait "${REC_PID}"
  rc=$?
  REC_PID=""
fi
if [ "${rc}" -ne 0 ] && [ "${rc}" -ne 130 ]; then
  echo -e "${RED}[WARN] 录包进程退出码 ${rc}${NC}" >&2
fi

# ---- 收尾 ----
echo -e "${YELLOW}[nav_debug] 停止 relay 并收尾快照...${NC}"
cleanup
trap - EXIT

if [ -d "${BAG_NAME}" ]; then
  post_snapshot_shell "bag_info" "ros2 bag info ${BAG_NAME}"
fi

cat >"${RUN_DIR}/notes.md" <<EOF
# 导航调试日志记录（真机）

- **时间**: $(date --iso-8601=seconds)
- **目录**: \`${RUN_DIR}\`
- **录包时长**: $([ "${DURATION}" -gt 0 ] && echo "${DURATION}s" || echo "手动停止 (Ctrl+C)")
- **标签**: ${LABEL:-（无）}
- **局部代价地图**: ${WITH_COSTMAP}

## 采集内容（核心三路 + 辅助链）

| 数据 | 话题 | 坐标系 |
|------|------|--------|
| 全局路径 | \`/plan\` | map |
| MPPI 最优局部轨迹 | \`/nav_debug/mppi_optimal_traj\`（来自 /trajectories 提取） | odom |
| 限速后速度指令 | \`/cmd_vel_limiter\` | base（数值） |
| 对照：限速前 | \`/cmd_vel\` | base（数值） |
| MPPI 跟踪参考段 | \`/transformed_global_plan\` | odom |
| ICP 定位矩阵/fitness | \`/odom2map\`, \`/localization_3d_confidence\` | map→odom |
| FastLIO 原始里程计 | \`/Odometry\` | camera_init |
| 模式切换 | \`/walk_mode\` \`/stand_mode\` 等 | 事件 |

## 现象记录（请填写）

- **目标**: (x=___, y=___, yaw=___)
- **异常发生时刻**（相对开录）: ___ s
- **现象描述**（如：侧前方行走→某点后原地踏步）:
- **当时操作**: 
- **RViz/终端表现**: 

## 后处理

\`\`\`bash
python3 ${SCRIPT_DIR}/nav_debug_plot.py ${RUN_DIR}
# 输出: analysis/map_overlay.png, analysis/cmd_vel.png, analysis/summary.md, analysis/data/*.csv
\`\`\`
EOF

echo ""
echo -e "${GREEN}[nav_debug] 采集完成${NC}"
if [ "${INT_RECEIVED}" = 1 ]; then
  echo "  （检测到 Ctrl+C, 已走完整收尾流程: bag metadata / 快照 / notes 均已写入）"
fi
echo "  目录: ${RUN_DIR}"
echo "  请填写: ${RUN_DIR}/notes.md"
if [ -d "${BAG_NAME}" ]; then
  echo "  录包:   ros2 bag info ${BAG_NAME}"
fi
echo "  复盘:   python3 ${SCRIPT_DIR}/nav_debug_plot.py ${RUN_DIR}"
