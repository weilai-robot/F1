#!/bin/bash
# ============================================================
# record_nav_test.sh — 真机导航定位测试录包（精简 topic）
#
# 只录定位分层诊断所需数据，不录点云/原始雷达（体积大、回放慢）。
# 用于区分：LIO odom 暴走 / Open3D map→odom 跳变 / TF 链断裂。
#
# 用法:
#   ./record_nav_test.sh                    # 录到 Ctrl+C
#   ./record_nav_test.sh -d 60              # 录 60 秒
#   ./record_nav_test.sh -d 30 pickup       # 带标签 pickup
#   ./record_nav_test.sh --nav -d 120 walk  # 含 /cmd_vel，走 Nav2 测试
#
# 输出:
#   F1/reports/nav_test_record/<timestamp>_<label>/
#     notes.md           — 测试元数据模板（请手填）
#     pre_snapshot/      — 开录前一次性快照
#     bag/               — rosbag2
#
# 前提: run_nav_real.sh 栈已起（livox + fastlio + nav2）
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
NAV_DIR="${ROOT_DIR}/navigation"
REPORT_ROOT="${ROOT_DIR}/reports/nav_test_record"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

DURATION=0          # 0 = until Ctrl+C
LABEL=""
WITH_NAV=false

usage() {
  sed -n '2,22p' "$0"
  exit "${1:-0}"
}

while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help) usage 0 ;;
    -d|--duration)
      DURATION="${2:?missing duration}"
      shift 2
      ;;
    --nav) WITH_NAV=true; shift ;;
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
BAG_DIR="${RUN_DIR}/bag"

# 定位分层最小集（不含点云 / 原始 Livox）
TOPICS=(
  /tf
  /tf_static
  /Odometry
  /odom
  /odom2map
  /localization_3d_confidence
)
if [ "${WITH_NAV}" = true ]; then
  TOPICS+=(/cmd_vel)
fi

mkdir -p "${SNAP_DIR}" "${BAG_DIR}"

echo -e "${GREEN}[record] 输出目录:${NC} ${RUN_DIR}"

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

echo -e "${YELLOW}[record] 检查 topic...${NC}"
MISSING=()
for t in "${TOPICS[@]}"; do
  if ! ros2 topic list 2>/dev/null | grep -qx "${t}"; then
    MISSING+=("${t}")
  fi
done
if [ ${#MISSING[@]} -gt 0 ]; then
  echo -e "${RED}[WARN] 以下 topic 当前不存在（录包仍会尝试，可能无数据）:${NC}" >&2
  printf '  %s\n' "${MISSING[@]}" >&2
fi

echo -e "${YELLOW}[record] 开录前快照...${NC}"
snapshot "topic_list" ros2 topic list -t
snapshot_shell "hz_odometry" "timeout 6s ros2 topic hz /Odometry"
snapshot_shell "confidence_once" "timeout 5s ros2 topic echo /localization_3d_confidence --once"
snapshot_shell "odom2map_once" "timeout 5s ros2 topic echo /odom2map --once"
snapshot_shell "tf_map_odom" "timeout 5s ros2 run tf2_ros tf2_echo map odom"
snapshot_shell "tf_odom_base_footprint" "timeout 5s ros2 run tf2_ros tf2_echo odom base_footprint"

cat >"${RUN_DIR}/notes.md" <<EOF
# 导航定位测试记录

- **时间**: $(date --iso-8601=seconds)
- **目录**: \`${RUN_DIR}\`
- **录包时长**: $([ "${DURATION}" -gt 0 ] && echo "${DURATION}s" || echo "手动停止 (Ctrl+C)")
- **标签**: ${LABEL:-（无）}
- **含 Nav2 cmd_vel**: ${WITH_NAV}

## 环境与安装

- [ ] 雷达安装: 小推车 / 已上机器人（高度、倒装角: ___）
- [ ] FastLIO 配置: F1_real_mid360.yaml / car_30_mid360_real.yaml
- [ ] 地图: car30_real_fastlio.yaml + car_30_real_map.pcd
- [ ] 场地: ___

## 测试内容

- [ ] 静止基线 / 抱起 / 轻推小车 / 原地踏步 / 短距离走 / Nav2 目标
- **操作描述**（做了什么、哪段时间）:


## 现象

- **是否飘飞**: 是 / 否
- **类型**: LIO odom 暴走 / map→odom 跳变 / 缓慢漂移 / 其他
- **大约时刻**（相对开录）: ___s
- **RViz 表现**: ___

## 回放诊断（bag 播完后）

\`\`\`bash
# LIO 是否在 odom 里暴走
ros2 bag play ${BAG_DIR}/* --clock
ros2 topic echo /Odometry --field pose.pose.position

# map→odom 是否跳变
ros2 topic echo /odom2map

# ICP 置信度
ros2 topic echo /localization_3d_confidence
\`\`\`

| 信号 | 正常 | 异常含义 |
|------|------|----------|
| \`/Odometry\` 静止时位移 | < 几 cm | LIO 在跟踪传感器运动或发散 |
| \`/odom2map\` | 缓慢变化 | 突然跳 = ICP 误匹配 |
| \`localization_3d_confidence\` | > 0.7 | 低分 = ICP 不可靠 |
EOF

echo -e "${YELLOW}[record] topics (${#TOPICS[@]}):${NC}"
printf '  %s\n' "${TOPICS[@]}"

BAG_NAME="${BAG_DIR}/nav_test"
RECORD_CMD=(ros2 bag record -o "${BAG_NAME}" "${TOPICS[@]}")

echo -e "${GREEN}[record] 开始录包 → ${BAG_NAME}${NC}"
if [ "${DURATION}" -gt 0 ]; then
  echo -e "${YELLOW}  时长 ${DURATION}s，到时自动停止${NC}"
  timeout --signal=INT "${DURATION}" "${RECORD_CMD[@]}" || {
    rc=$?
    if [ "${rc}" -eq 124 ]; then
      echo -e "${GREEN}[record] 已达 ${DURATION}s，录包结束${NC}"
    else
      exit "${rc}"
    fi
  }
else
  echo -e "${YELLOW}  Ctrl+C 停止${NC}"
  "${RECORD_CMD[@]}"
fi

# 录包 metadata
if [ -d "${BAG_NAME}" ]; then
  snapshot_shell "bag_info" "ros2 bag info ${BAG_NAME}"
fi

cat >>"${RUN_DIR}/notes.md" <<EOF

## 录包结果

- **结束时间**: $(date --iso-8601=seconds)
- **bag 路径**: \`${BAG_NAME}\`
EOF

echo ""
echo -e "${GREEN}[record] 完成${NC}"
echo "  目录: ${RUN_DIR}"
echo "  请填写: ${RUN_DIR}/notes.md"
echo "  回放:   ros2 bag play ${BAG_NAME}"
