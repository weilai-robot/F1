#!/bin/bash
# ============================================================
# run_real_all.sh — 真机「大脑(导航) + 小脑(运控)」联合一键启动
#
# 特点:
#   - 小脑使用 x1_cfg_real_nav.yaml: 不加载 JoyStickModule,
#     导航期间 /cmd_vel_limiter 唯一来源为 Nav2 → odom_bridge → 小脑
#   - setcap 统一在 tmux 之前完成 (sudo 密码可正常输入)
#   - 可选 --build: 先重编 motion_control + navigation 再启动
#
# tmux 窗口:
#   livox    — Livox MID-360 驱动
#   fastlio  — FastLIO2 里程计
#   nav2     — odom_bridge + open3d_loc + Nav2 (含 cmd_vel 限幅中继)
#   aimrt    — 小脑 (no-joy 配置; 模式由外部脚本切换)
#   record   — 检查 / 录包提示
#
# 用法:
#   ./run_real_all.sh                 # 直接启动 (需已构建)
#   ./run_real_all.sh --build         # 先构建 (build_all.sh) 再启动
#   ./run_real_all.sh --build-only    # 只构建, 不启动
#   ./run_real_all.sh --no-rviz       # FastLIO 不弹 RViz
#
# 启动后操作 (另开终端; 用前先看屏上提示):
#   1. ./switch_x1_mode.sh ready                   # zero → stand → walk
#   2. ./send_nav_goal_real.sh --no-mode 1.0 0.0   # 发导航目标
#   3. 急停: ./send_nav_goal_real.sh --stand-only  或物理急停按钮
#
# ⚠ 注意:
#   - 本模式小脑不加载手柄模块: 手柄按键/摇杆全部无效 (含 LT/RT)
#   - 导航期间勿运行 sim_keyboard_ctrl.py / auto_walk_ctrl.py 等直发速度脚本
#   - 停止顺序: 先 /stand_mode, 后让 /cmd_vel 归零 (脚本已内置)
#   - 真机首次联调请保持机器人受保护, 人手不离急停
# ============================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
NAV_DIR="${ROOT_DIR}/navigation"
BUILD_DIR="${ROOT_DIR}/build"
SESSION_NAME="f1_real_all"

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

# ── 参数解析 ──────────────────────────────────────────────
DO_BUILD=false
BUILD_ONLY=false
RVIZ_ARG="rviz:=true"

for arg in "$@"; do
  case "$arg" in
    --build)      DO_BUILD=true ;;
    --build-only) DO_BUILD=true; BUILD_ONLY=true ;;
    --no-rviz)    RVIZ_ARG="rviz:=false" ;;
    --rviz)       RVIZ_ARG="rviz:=true" ;;
    -h|--help)
      sed -n '2,35p' "$0"
      exit 0
      ;;
    *)
      echo -e "${RED}[ERROR] 未知参数: ${arg}${NC}"
      exit 1
      ;;
  esac
done

# ── 可选: 先构建（motion_control + navigation）─────────────
if [ "${DO_BUILD}" = true ]; then
  echo -e "${GREEN}[run_real_all] 构建 motion_control + navigation ...${NC}"
  "${SCRIPT_DIR}/build_all.sh"
  echo -e "${GREEN}  ✓ 构建完成${NC}"
fi
if [ "${BUILD_ONLY}" = true ]; then
  echo -e "${GREEN}[run_real_all] --build-only: 仅构建, 退出${NC}"
  exit 0
fi

echo -e "${GREEN}[run_real_all] 启动真机「大脑 + 小脑(no-joy)」...${NC}"

# ── tmux 依赖 ─────────────────────────────────────────────
if ! command -v tmux >/dev/null 2>&1; then
  echo -e "${RED}[ERROR] 未安装 tmux${NC}"
  echo -e "  sudo apt install tmux"
  exit 1
fi

# ── ROS2 setup（优先 scripts/ros2_source.sh，再回退探测）──
if [ -f "${SCRIPT_DIR}/ros2_source.sh" ]; then
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
  exit 1
fi

echo -e "${GREEN}  ROS2: ${ROS_SETUP_BASH}${NC}"
source "${ROS_SETUP_BASH}"

# ── navigation 侧检查 ─────────────────────────────────────
if [ ! -f "${NAV_DIR}/install/setup.bash" ]; then
  echo -e "${RED}[ERROR] navigation workspace 未构建${NC}"
  echo -e "  请先运行: ./build_nav.sh"
  exit 1
fi
source "${NAV_DIR}/install/setup.bash"

ros_pkg_exists() {
  ros2 pkg prefix "$1" >/dev/null 2>&1
}

for pkg in fast_lio humanoid_sim open3d_loc livox_ros_driver2; do
  if ! ros_pkg_exists "${pkg}"; then
    echo -e "${RED}[ERROR] 缺少 ROS2 package: ${pkg}${NC}"
    echo -e "  请先: ./build_nav.sh"
    exit 1
  fi
done

if ! ros_pkg_exists nav2_bringup; then
  echo -e "${RED}[ERROR] 缺少 nav2_bringup${NC}"
  echo -e "  sudo apt install ros-humble-nav2-bringup"
  exit 1
fi

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

# ── 小脑侧检查：定位 aimrt 运行目录（no-joy 配置）──────────
AIMRT_DIR=""
for d in "${BUILD_DIR}" "${ROOT_DIR}/motion_control"; do
  if [ -f "${d}/aimrt_main" ] && \
     [ -f "${d}/cfg/x1_cfg_real_nav.yaml" ] && \
     [ -f "${d}/cfg/control_module/rl_x1.yaml" ]; then
    AIMRT_DIR="$d"
    break
  fi
done

if [ -z "${AIMRT_DIR}" ]; then
  echo -e "${RED}[ERROR] 未找到可用的小脑运行目录（缺 aimrt_main / x1_cfg_real_nav.yaml / control cfg）${NC}"
  echo -e "  已检查: ${BUILD_DIR}, ${ROOT_DIR}/motion_control"
  echo -e "  请先构建: ./build_all.sh   （或用本脚本 --build）"
  exit 1
fi
echo -e "${GREEN}  小脑目录: ${AIMRT_DIR}${NC}"

if grep -qE '^\s*-\s*JoyStickModule\s*$' "${AIMRT_DIR}/cfg/x1_cfg_real_nav.yaml"; then
  echo -e "${YELLOW}[WARN] x1_cfg_real_nav.yaml 仍启用 JoyStickModule —— 导航期间手柄会抢占 /cmd_vel_limiter${NC}"
  echo -e "${YELLOW}       请确认配置; 5 秒内 Ctrl+C 可中止...${NC}"
  sleep 5
fi
if ! grep -qE '^\s*-\s*ControlModule\s*$' "${AIMRT_DIR}/cfg/x1_cfg_real_nav.yaml"; then
  echo -e "${RED}[ERROR] x1_cfg_real_nav.yaml 未启用 ControlModule，配置异常${NC}"
  exit 1
fi

# ── cap_net_raw 预检（在 tmux 之外完成, sudo 密码可正常输入）──
if [ "${SKIP_SETCAP:-0}" = "1" ]; then
  echo -e "${YELLOW}[run_real_all] SKIP_SETCAP=1, 跳过 cap_net_raw 检查${NC}"
else
  NEED_SETCAP=1
  if command -v getcap >/dev/null 2>&1 && getcap "${AIMRT_DIR}/aimrt_main" 2>/dev/null | grep -q "cap_net_raw"; then
    NEED_SETCAP=0
    echo -e "${GREEN}  ✓ aimrt_main cap_net_raw 已就绪${NC}"
  fi
  if [ "${NEED_SETCAP}" = 1 ]; then
    echo -e "${YELLOW}[run_real_all] 设置 aimrt_main cap_net_raw（需要 sudo）...${NC}"
    if ! sudo setcap cap_net_raw=ep "${AIMRT_DIR}/aimrt_main"; then
      echo -e "${RED}[ERROR] setcap 失败 —— 小脑 EtherCAT 将无法通信${NC}"
      echo -e "${YELLOW}  手动执行后重试: sudo setcap cap_net_raw=ep ${AIMRT_DIR}/aimrt_main${NC}"
      echo -e "${YELLOW}  如已通过其他方式授权, 可用 SKIP_SETCAP=1 跳过本检查${NC}"
      exit 1
    fi
    echo -e "${GREEN}  ✓ cap_net_raw 设置完成${NC}"
  fi
fi

# ── 检查是否已有同名 session ───────────────────────────────
if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
  echo -e "${YELLOW}[WARN] 已存在 tmux session: ${SESSION_NAME}${NC}"
  echo -e "  关闭: tmux kill-session -t ${SESSION_NAME}"
  exit 1
fi

# ── 每个窗口的 source 前缀（可覆盖 TMUX_SOURCE）────────────
if [ -z "${TMUX_SOURCE:-}" ]; then
  TMUX_SOURCE="source ${ROS_SETUP_BASH} && source ${NAV_DIR}/install/setup.bash"
fi

# --- [livox] ---
tmux new-session -d -s "${SESSION_NAME}" -n "livox"
SESSION_CREATED=1
echo -e "${GREEN}  [livox] Livox MID-360${NC}"
tmux send-keys -t "${SESSION_NAME}:livox" "${TMUX_SOURCE}" Enter
tmux send-keys -t "${SESSION_NAME}:livox" "ros2 launch livox_ros_driver2 msg_MID360_launch.py" Enter
echo -e "${YELLOW}  等待雷达数据 (5s)...${NC}"
sleep 5

# --- [fastlio] ---
tmux new-window -t "${SESSION_NAME}" -n "fastlio"
echo -e "${GREEN}  [fastlio] FastLIO2 (F1_real_mid360.yaml)${NC}"
tmux send-keys -t "${SESSION_NAME}:fastlio" "${TMUX_SOURCE}" Enter
tmux send-keys -t "${SESSION_NAME}:fastlio" "ros2 launch fast_lio mapping_real.launch.py config_file:=F1_real_mid360.yaml ${RVIZ_ARG}" Enter
echo -e "${YELLOW}  等待 LIO 收敛 (5s)...${NC}"
sleep 5

# --- [nav2] ---
tmux new-window -t "${SESSION_NAME}" -n "nav2"
echo -e "${GREEN}  [nav2] navigation_real.launch.py${NC}"
tmux send-keys -t "${SESSION_NAME}:nav2" "${TMUX_SOURCE}" Enter
tmux send-keys -t "${SESSION_NAME}:nav2" "ros2 launch humanoid_sim navigation_real.launch.py" Enter
echo -e "${YELLOW}  等待 Nav2 / ICP (5s)...${NC}"
sleep 5

# --- [aimrt] 小脑（no-joy 配置）---
tmux new-window -t "${SESSION_NAME}" -n "aimrt"
echo -e "${GREEN}  [aimrt] 小脑 aimrt_main (x1_cfg_real_nav.yaml, 无手柄)${NC}"
tmux send-keys -t "${SESSION_NAME}:aimrt" "cd ${AIMRT_DIR} && bash ./run_real_nav.sh" Enter
sleep 3

# --- [record] ---
tmux new-window -t "${SESSION_NAME}" -n "record"
echo -e "${GREEN}  [record] 检查 / 录包提示${NC}"
tmux send-keys -t "${SESSION_NAME}:record" "source ${ROS_SETUP_BASH}" Enter
tmux send-keys -t "${SESSION_NAME}:record" \
  "echo '=== 真机联合模式检查 ===
  工作目录: cd ${SCRIPT_DIR}
  小脑就绪: [aimrt] 窗口出现 Init succeeded
  模式切换: ./switch_x1_mode.sh ready
  发目标:   ./send_nav_goal_real.sh --no-mode 1.0 0.0
  急停:     ./send_nav_goal_real.sh --stand-only  或物理急停
  速度链路: ros2 topic hz /cmd_vel_limiter   (导航中约 10Hz)
  发布者:   ros2 topic info -v /cmd_vel_limiter  (应只有 odom_bridge)
  录包:     ./record_nav_test.sh --nav -d 120 walk'" Enter

# --- 完成 ---
echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN} 大脑(导航) + 小脑(no-joy) 已启动${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo " tmux 窗口:"
echo "   livox     - Livox MID-360"
echo "   fastlio   - FastLIO2 (F1_real_mid360.yaml)"
echo "   nav2      - odom_bridge + open3d_loc + Nav2"
echo "   aimrt     - 小脑 (x1_cfg_real_nav.yaml, 不加载手柄)"
echo "   record    - 检查 / 录包提示"
echo ""
echo " 附加: tmux attach -t ${SESSION_NAME}"
echo " 关闭: tmux kill-session -t ${SESSION_NAME}"
echo ""
echo -e "${YELLOW} 操作流程（另开终端执行）:${NC}"
echo -e "   0. 等 [aimrt] 窗口出现 'Init succeeded'; 等 TF: map→odom→base_footprint"
echo -e "   1. cd ${SCRIPT_DIR}"
echo -e "   2. 切模式: ./switch_x1_mode.sh ready"
echo -e "   3. 发目标: ./send_nav_goal_real.sh --no-mode 1.0 0.0"
echo -e "   4. 急停:   ./send_nav_goal_real.sh --stand-only  或物理急停"
echo ""
echo -e "${YELLOW} 注意:${NC}"
echo -e "   - 本模式小脑不加载手柄模块: 手柄按键/摇杆全部无效（含 LT/RT）"
echo -e "   - 速度唯一来源: Nav2 → odom_bridge → /cmd_vel_limiter"
echo -e "   - 导航期间勿运行 sim_keyboard_ctrl.py / auto_walk_ctrl.py 等直发速度脚本"
echo ""
