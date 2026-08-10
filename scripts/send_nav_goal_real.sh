#!/bin/bash
# ============================================================
# 真机一键导航: 切模式 + 发 NavigateToPose — send_nav_goal_real.sh
#
# 与仿真 send_nav_goal.sh 的差异:
#   - source AimRT ros2_setup（与 sim_control 一致，否则小脑收不到 mode）
#   - 不调用 nav_test_runner（依赖 /mujoco/ground_truth，真机没有）
#   - 用 ros2 action send_goal 发目标；结束后自动 stand（防摔倒）
#   - 默认不跑仿真 batch / 指标报告
#
# 前提:
#   1. AimRT: bash run.sh（另开终端）
#   2. 大脑: ./run_nav_real.sh（livox + fastlio + nav2）
#   3. TF map→odom→base_footprint 已通；ICP 已锁定
#
# 用法:
#   ./send_nav_goal_real.sh 1.0 0.0              # walk → goal → stand
#   ./send_nav_goal_real.sh 1.0 0.0 90           # 终点朝向 90°
#   ./send_nav_goal_real.sh 1.0 0.0 0 180        # 超时 180s（默认 120）
#   ./send_nav_goal_real.sh --ready 1.0 0.0     # zero→stand→walk → goal → stand
#   ./send_nav_goal_real.sh --walk-only         # 只切 walk
#   ./send_nav_goal_real.sh --stand-only        # 只切 stand（紧急停步）
#   ./send_nav_goal_real.sh --no-mode 1.0 0.0   # 已 walk，只发 goal
# ============================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
NAV_DIR="${ROOT_DIR}/navigation"
SWITCH_MODE="${SCRIPT_DIR}/switch_x1_mode.sh"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

DO_READY=false
DO_WALK=true
DO_STAND_AFTER=true
WALK_ONLY=false
STAND_ONLY=false
TIMEOUT=120

usage() {
  sed -n '2,28p' "$0"
  exit "${1:-0}"
}

# ── 参数 ──
ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help) usage 0 ;;
    --ready) DO_READY=true; DO_WALK=false; shift ;;
    --no-mode) DO_READY=false; DO_WALK=false; shift ;;
    --no-stand-after) DO_STAND_AFTER=false; shift ;;
    --walk-only) WALK_ONLY=true; shift ;;
    --stand-only) STAND_ONLY=true; shift ;;
    --*)
      echo -e "${RED}[ERROR] 未知选项: $1${NC}"
      usage 1
      ;;
    *)
      ARGS+=("$1")
      shift
      ;;
  esac
done

if [ ! -x "${SWITCH_MODE}" ]; then
  echo -e "${RED}[ERROR] 缺少 ${SWITCH_MODE}${NC}"
  exit 1
fi

# ── 仅切模式 ──
if [ "${STAND_ONLY}" = true ]; then
  exec "${SWITCH_MODE}" stand
fi
if [ "${WALK_ONLY}" = true ]; then
  exec "${SWITCH_MODE}" walk
fi

if [ "${#ARGS[@]}" -lt 2 ]; then
  echo -e "${RED}[ERROR] 缺少目标坐标 x y${NC}"
  usage 1
fi

GOAL_X="${ARGS[0]}"
GOAL_Y="${ARGS[1]}"
YAW_DEG="${ARGS[2]:-0}"
TIMEOUT="${ARGS[3]:-${TIMEOUT}}"

# ── ROS2 + navigation（发 action 需要；mode 由 switch_x1_mode 自带 AimRT setup）──
if [ -f "${SCRIPT_DIR}/ros2_source.sh" ]; then
  # shellcheck source=/dev/null
  source "${SCRIPT_DIR}/ros2_source.sh"
elif [ -z "${ROS_SETUP_BASH:-}" ]; then
  if   [ -f "${CONDA_PREFIX:-}/ros_humble/setup.bash" ]; then ROS_SETUP_BASH="${CONDA_PREFIX}/ros_humble/setup.bash"
  elif [ -f "${CONDA_PREFIX:-}/setup.bash" ]; then ROS_SETUP_BASH="${CONDA_PREFIX}/setup.bash"
  elif [ -f /opt/ros/humble/setup.bash ]; then ROS_SETUP_BASH="/opt/ros/humble/setup.bash"
  fi
  if [ -n "${ROS_SETUP_BASH:-}" ]; then
    # shellcheck source=/dev/null
    source "${ROS_SETUP_BASH}"
  fi
fi

if [ ! -f "${NAV_DIR}/install/setup.bash" ]; then
  echo -e "${RED}[ERROR] navigation 未构建: ${NAV_DIR}/install/setup.bash${NC}"
  exit 1
fi
# shellcheck source=/dev/null
source "${NAV_DIR}/install/setup.bash"

# ── 结束/中断 → stand（真机安全；先 stand 再让 cmd_vel 自然归零）──
STAND_DONE=0
safe_stand() {
  if [ "${DO_STAND_AFTER}" != true ]; then
    return 0
  fi
  if [ "${STAND_DONE}" = 1 ]; then
    return 0
  fi
  STAND_DONE=1
  echo ""
  echo -e "${YELLOW}[safe] → stand_mode（导航结束/中断，先站立）${NC}"
  "${SWITCH_MODE}" stand || true
}
trap safe_stand EXIT INT TERM

# ── 切模式 ──
if [ "${DO_READY}" = true ]; then
  echo -e "${GREEN}[1/2] ready: zero → stand → walk${NC}"
  "${SWITCH_MODE}" ready
elif [ "${DO_WALK}" = true ]; then
  echo -e "${GREEN}[1/2] walk_mode${NC}"
  "${SWITCH_MODE}" walk
  echo -e "${YELLOW}      等待行走稳定 (2s)...${NC}"
  sleep 2
else
  echo -e "${YELLOW}[1/2] 跳过切模式（--no-mode）${NC}"
fi

# ── 发 NavigateToPose（map 系，与真机 2D 地图一致）──
YAW_RAD=$(python3 -c "import math,sys; print(math.radians(float(sys.argv[1])))" "${YAW_DEG}")
read -r QZ QW <<EOF
$(python3 -c "import math,sys; y=float(sys.argv[1]); print(math.sin(y/2.0), math.cos(y/2.0))" "${YAW_RAD}")
EOF

echo -e "${GREEN}[2/2] 发送导航目标: map=(${GOAL_X}, ${GOAL_Y}, yaw=${YAW_DEG}°) timeout=${TIMEOUT}s${NC}"
echo -e "${YELLOW}      确认目标在 car30_real_fastlio 地图自由区；人在旁护着${NC}"

# 粗查 navigate_to_pose 是否在线
if ! ros2 action list 2>/dev/null | grep -q '/navigate_to_pose'; then
  echo -e "${RED}[ERROR] 未找到 action /navigate_to_pose — 确认 run_nav_real.sh / Nav2 已启动${NC}"
  exit 1
fi

GOAL_YAML="{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: ${GOAL_X}, y: ${GOAL_Y}, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: ${QZ}, w: ${QW}}
    }
  }
}"

set +e
timeout --signal=INT "${TIMEOUT}" \
  ros2 action send_goal --feedback /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "${GOAL_YAML}"
RC=$?
set -e

if [ "${RC}" -eq 0 ]; then
  echo -e "${GREEN}导航 action 结束 (ok / 已被 server 完成)${NC}"
elif [ "${RC}" -eq 124 ]; then
  echo -e "${YELLOW}导航超时 (${TIMEOUT}s)，将切 stand${NC}"
else
  echo -e "${YELLOW}导航中断或失败 (rc=${RC})，将切 stand${NC}"
fi

# trap EXIT 会调 safe_stand
exit "${RC}"
