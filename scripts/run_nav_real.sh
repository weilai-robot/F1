#!/bin/bash
# ============================================================
# 真机导航一键启动 — run_nav_real.sh（仅大脑）
# 对齐 docs/x1_real_nav_first_bringup.md + run_mujoco_nav.sh 结构
#
# tmux 窗口:
#   [0] livox        — Livox MID-360 驱动
#   [1] fastlio      — FastLIO2 (F1_real_mid360.yaml)
#   [2] nav2         — navigation_real (odom_bridge + open3d_loc + Nav2)
#   [record]         — rosbag / 监控提示（手动启动）
#   nav_state_manager — 已禁用，真机首次请手柄/手动 topic 切模式
#
# 小脑 AimRT 请另开终端手动启动（避免 tmux 里 sudo setcap 卡密码）:
#   cd <aimrt_dir>   # 通常 F1/build 或 motion_control/install/linux/bin
#   sudo setcap cap_net_raw=ep ./aimrt_main   # 若尚未设置
#   bash ./run.sh
#
# 用法:
#   ./run_nav_real.sh                 # 导航大脑（不含 AimRT / nav_state）
#   ./run_nav_real.sh                 # 导航大脑（FastLIO 默认弹 RViz）
#   ./run_nav_real.sh --no-rviz       # FastLIO 不弹 RViz
#
# 前置:
#   1. ./build_nav.sh              （navigation workspace）
#   2. Mid360 已联网/连接
#   3. 另开终端已启动 AimRT 小脑
# ============================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
NAV_DIR="${ROOT_DIR}/navigation"
BUILD_DIR="${ROOT_DIR}/build"
SESSION_NAME="f1_nav_real"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

SESSION_CREATED=0
launch_cleanup() {
  local rc=$?
  if [ "${SESSION_CREATED}" = 1 ] && [ "${rc}" -ne 0 ]; then
    echo ""
    echo -e "${RED}[FAIL] 启动中断 (rc=${rc}).${NC}" >&2
    echo -e "${YELLOW}  tmux attach -t ${SESSION_NAME}${NC}" >&2
    echo -e "${YELLOW}  清理: tmux kill-session -t ${SESSION_NAME}${NC}" >&2
  fi
}
trap launch_cleanup EXIT

# --- 参数 ---
# 默认弹 FastLIO RViz（调试用）；不需要时传 --no-rviz
RVIZ_ARG="rviz:=true"
for arg in "$@"; do
  case "$arg" in
    --no-aimrt) ;;       # 兼容旧参数：小脑已默认不启
    --no-nav-state) ;;   # 兼容旧参数：nav_state 已默认不启
    --no-rviz) RVIZ_ARG="rviz:=false" ;;
    --rviz) RVIZ_ARG="rviz:=true" ;;
    -h|--help)
      sed -n '2,28p' "$0"
      exit 0
      ;;
    *)
      echo -e "${RED}[ERROR] 未知参数: ${arg}${NC}"
      exit 1
      ;;
  esac
done

echo -e "${GREEN}[run_nav_real] 启动真机导航大脑（不含 AimRT / nav_state）...${NC}"
echo -e "${YELLOW}  请确认小脑已在另一终端用 run.sh 启动；模式请手柄/手动切换${NC}"

# ── ROS2 setup（优先 scripts/ros2_source.sh，再回退探测）──
if [ -f "${SCRIPT_DIR}/ros2_source.sh" ]; then
  # shellcheck source=/dev/null
  source "${SCRIPT_DIR}/ros2_source.sh"
  ROS_SETUP_BASH="${ROS2_FOUND:-${ROS_SETUP_BASH:-}}"
fi

if [ -z "${ROS_SETUP_BASH:-}" ]; then
  if [ -f "${CONDA_PREFIX:-}/ros_humble/setup.bash" ]; then
    ROS_SETUP_BASH="${CONDA_PREFIX}/ros_humble/setup.bash"
  elif [ -f "${CONDA_PREFIX:-}/setup.bash" ]; then
    ROS_SETUP_BASH="${CONDA_PREFIX}/setup.bash"
  elif [ -f /opt/ros/humble/setup.bash ]; then
    ROS_SETUP_BASH="/opt/ros/humble/setup.bash"
  fi
fi

if [ -z "${ROS_SETUP_BASH:-}" ] || [ ! -f "${ROS_SETUP_BASH}" ]; then
  echo -e "${RED}[ERROR] 未找到 ROS2 setup.bash${NC}"
  echo -e "  请设置: export ROS_SETUP_BASH=/path/to/setup.bash"
  echo -e "  或:     export ROS2_SETUP_PATH=/path/to/setup.bash"
  exit 1
fi

echo -e "${GREEN}  ROS2: ${ROS_SETUP_BASH}${NC}"
# shellcheck source=/dev/null
source "${ROS_SETUP_BASH}"

if [ ! -f "${NAV_DIR}/install/setup.bash" ]; then
  echo -e "${RED}[ERROR] navigation workspace 未构建${NC}"
  echo -e "  请先运行: ./build_nav.sh"
  exit 1
fi
# shellcheck source=/dev/null
source "${NAV_DIR}/install/setup.bash"

ros_pkg_exists() {
  ros2 pkg prefix "$1" >/dev/null 2>&1
}

print_missing_ros_pkg_help() {
  local pkg="$1"
  echo -e "${RED}[ERROR] 缺少 ROS2 package: ${pkg}${NC}"
  echo -e "  当前 ROS2: ${ROS_SETUP_BASH}"
  echo -e "  请先: ./build_nav.sh"
}

for pkg in fast_lio humanoid_sim open3d_loc livox_ros_driver2; do
  if ! ros_pkg_exists "${pkg}"; then
    print_missing_ros_pkg_help "${pkg}"
    exit 1
  fi
done

if ! ros_pkg_exists nav2_bringup; then
  echo -e "${RED}[ERROR] 缺少 nav2_bringup${NC}"
  echo -e "  sudo apt install ros-humble-nav2-bringup"
  exit 1
fi

# FastLIO 真机配置必须已 install
FAST_LIO_PREFIX="$(ros2 pkg prefix fast_lio)"
FAST_LIO_CFG="${FAST_LIO_PREFIX}/share/fast_lio/config/F1_real_mid360.yaml"
if [ ! -f "${FAST_LIO_CFG}" ]; then
  echo -e "${RED}[ERROR] FastLIO2 真机配置未安装: ${FAST_LIO_CFG}${NC}"
  echo -e "  请重新构建: ./build_nav.sh --packages-select fast_lio"
  exit 1
fi

HUMANOID_SIM_PREFIX="$(ros2 pkg prefix humanoid_sim)"
ODOM_BRIDGE_EXE="${HUMANOID_SIM_PREFIX}/lib/humanoid_sim/odom_bridge.py"
if [ ! -x "${ODOM_BRIDGE_EXE}" ]; then
  echo -e "${RED}[ERROR] odom_bridge.py 未安装: ${ODOM_BRIDGE_EXE}${NC}"
  echo -e "  请重新构建: ./build_nav.sh --packages-select humanoid_sim"
  exit 1
fi

NAV2_REAL_CFG="${HUMANOID_SIM_PREFIX}/share/humanoid_sim/config/nav2_real.yaml"
if [ ! -f "${NAV2_REAL_CFG}" ]; then
  echo -e "${RED}[ERROR] nav2_real.yaml 未安装: ${NAV2_REAL_CFG}${NC}"
  echo -e "  请重新构建: ./build_nav.sh --packages-select humanoid_sim"
  exit 1
fi

# ---------------------------------------------------------------------------
# AimRT 小脑启动已禁用：请另开终端手动 bash ./run.sh
# （tmux send-keys 无法交互输入 sudo 密码；大脑/小脑分开更稳）
#
# AIMRT_CANDIDATES=(
#   "${BUILD_DIR}"
#   "${ROOT_DIR}/motion_control/install/linux/bin"
# )
# ENABLE_AIMRT=true
# AIMRT_DIR=""
# if [ "${ENABLE_AIMRT}" = true ]; then
#   for d in "${AIMRT_CANDIDATES[@]}"; do
#     if [ -x "${d}/aimrt_main" ] && [ -f "${d}/cfg/x1_cfg.yaml" ]; then
#       AIMRT_DIR="${d}"; break
#     fi
#   done
#   ...
#   tmux new-session -d -s "${SESSION_NAME}" -n "aimrt"
#   tmux send-keys -t "${SESSION_NAME}:aimrt" \
#     "cd ${AIMRT_DIR} && ./aimrt_main --cfg_file_path=./cfg/x1_cfg.yaml" Enter
# fi
# ---------------------------------------------------------------------------

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
  echo -e "${YELLOW}[WARN] 已存在 tmux session: ${SESSION_NAME}${NC}"
  echo -e "  关闭: tmux kill-session -t ${SESSION_NAME}"
  exit 1
fi

# 每个窗口的 source 前缀（与仿真脚本一致：可覆盖 TMUX_SOURCE）
if [ -z "${TMUX_SOURCE:-}" ]; then
  TMUX_SOURCE="source ${ROS_SETUP_BASH} && source ${NAV_DIR}/install/setup.bash"
fi

# --- [0] Livox ---
tmux new-session -d -s "${SESSION_NAME}" -n "livox"
SESSION_CREATED=1
echo -e "${GREEN}  [livox] Livox MID-360${NC}"
tmux send-keys -t "${SESSION_NAME}:livox" "${TMUX_SOURCE}" Enter
tmux send-keys -t "${SESSION_NAME}:livox" \
  "ros2 launch livox_ros_driver2 msg_MID360_launch.py" Enter
echo -e "${YELLOW}  等待雷达数据 (5s)...${NC}"
sleep 5

# --- FastLIO2 ---
tmux new-window -t "${SESSION_NAME}" -n "fastlio"
echo -e "${GREEN}  [fastlio] FastLIO2 (F1_real_mid360.yaml)${NC}"
tmux send-keys -t "${SESSION_NAME}:fastlio" "${TMUX_SOURCE}" Enter
tmux send-keys -t "${SESSION_NAME}:fastlio" \
  "ros2 launch fast_lio mapping_real.launch.py config_file:=F1_real_mid360.yaml ${RVIZ_ARG}" Enter
echo -e "${YELLOW}  等待 LIO 收敛 (5s)...${NC}"
sleep 5

# --- Nav2 real (含 odom_bridge + open3d_loc) ---
tmux new-window -t "${SESSION_NAME}" -n "nav2"
echo -e "${GREEN}  [nav2] navigation_real.launch.py${NC}"
tmux send-keys -t "${SESSION_NAME}:nav2" "${TMUX_SOURCE}" Enter
tmux send-keys -t "${SESSION_NAME}:nav2" \
  "ros2 launch humanoid_sim navigation_real.launch.py" Enter
echo -e "${YELLOW}  等待 Nav2 / ICP (5s)...${NC}"
sleep 5

# --- nav_state_manager（已禁用：真机首次请人手切模式）---
# if true; then
#   tmux new-window -t "${SESSION_NAME}" -n "nav_state"
#   echo -e "${GREEN}  [nav_state] nav_state_manager${NC}"
#   tmux send-keys -t "${SESSION_NAME}:nav_state" "${TMUX_SOURCE}" Enter
#   tmux send-keys -t "${SESSION_NAME}:nav_state" \
#     "ros2 launch humanoid_sim nav_state_manager.launch.py startup_delay_s:=3.0 stand_hold_s:=3.0 cmd_vel_timeout_s:=3.0" Enter
# fi

# --- record 提示窗 ---
tmux new-window -t "${SESSION_NAME}" -n "record"
echo -e "${GREEN}  [record] 数据采集提示（手动）${NC}"
tmux send-keys -t "${SESSION_NAME}:record" "source ${ROS_SETUP_BASH}" Enter
tmux send-keys -t "${SESSION_NAME}:record" \
  "echo '=== 真机导航检查 / 录包 ===
  TF:   ros2 run tf2_ros tf2_echo map odom
        ros2 run tf2_ros tf2_echo odom base_footprint
  LIO:  ros2 topic hz /Odometry
  Mode: ros2 topic info /stand_mode -v
  手动切模式 (须 source AimRT ros2_setup，用本仓库脚本):
    ./switch_x1_mode.sh stand
    ./switch_x1_mode.sh walk
    ./switch_x1_mode.sh ready   # zero→stand→walk
  停止: ./switch_x1_mode.sh stand ，再让 /cmd_vel 归零
  录包: cd ${SCRIPT_DIR} && ./record_nav_test.sh -d 60 <label>
        ./record_nav_test.sh --nav -d 120 walk   # 含 cmd_vel'" Enter

# --- 完成 ---
echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN} 真机导航大脑已启动${NC}"
echo -e "${GREEN} （AimRT / nav_state 需手动，未自动拉起）${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo " tmux 窗口:"
echo "   livox     - Livox MID-360"
echo "   fastlio   - FastLIO2 (F1_real_mid360.yaml)"
echo "   nav2      - odom_bridge + open3d_loc + Nav2"
echo "   record    - 检查 / 录包提示"
echo ""
echo " 附加: tmux attach -t ${SESSION_NAME}"
echo " 关闭: tmux kill-session -t ${SESSION_NAME}"
echo ""
echo -e "${YELLOW} 操作流程:${NC}"
echo -e "   0. 另开终端启动 AimRT: cd build && bash ./run.sh"
echo -e "   1. 确认 /livox/lidar、/Odometry 有数据"
echo -e "   2. 确认 TF: map→odom→base_footprint"
echo -e "   3. 切模式+发目标: ./send_nav_goal_real.sh 1.0 0.0"
echo -e "      或分步: ./switch_x1_mode.sh ready  →  ./send_nav_goal_real.sh --no-mode 1.0 0.0"
echo -e "   4. 紧急停步: ./send_nav_goal_real.sh --stand-only"
echo -e "   5. 地图: car30_real_fastlio (2D) + car_30_real_map.pcd (ICP)"
echo ""
echo -e "${YELLOW} XP: experience-aimrt-ros2-mode-sub-topics, fact-biped-stand-before-cmd-vel-zero${NC}"
