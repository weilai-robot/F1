#!/bin/bash
# ============================================================
# sim2sim_test.sh — 全自动 Sim2Sim 导航验证 (一键执行)
#
# 自动完成: 构建 → 启动全栈仿真 → 跑测试场景 → 采集指标 → 出报告 → 清理
#
# 用法:
#   scripts/sim2sim_test.sh                  # 默认: 构建 + 全部6场景 + 报告
#   scripts/sim2sim_test.sh --no-build       # 跳过构建 (已有产物时)
#   scripts/sim2sim_test.sh --scenarios A,C  # 只跑指定场景
#   scripts/sim2sim_test.sh --single 5.0 0.0 # 跑单点目标
#   scripts/sim2sim_test.sh --report-dir reports/$(date +%Y%m%d)
#
# 产出:
#   reports/<scenario>_<timestamp>.md   — 单场景可读报告
#   reports/<scenario>_<timestamp>.json — 结构化结果
#   reports/batch_summary_<timestamp>.json — 批量汇总
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
NAV_DIR="${ROOT_DIR}/navigation"
BUILD_DIR="${ROOT_DIR}/build"

# --- 颜色 ---
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

SESSION_NAME="f1_sim_nav"
REPORT_DIR="${ROOT_DIR}/reports"
DO_BUILD=true
SCENARIOS=""
SINGLE_GOAL=""
READINESS_TIMEOUT=120  # 等待全栈就绪的超时秒数

# --- 参数解析 ---
while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-build)    DO_BUILD=false; shift ;;
        --scenarios)   SCENARIOS="$2"; shift 2 ;;
        --single)      SINGLE_GOAL="$2 $3"; shift 3 ;;
        --report-dir)  REPORT_DIR="$2"; shift 2 ;;
        --help|-h)
            head -22 "$0" | tail -20
            exit 0 ;;
        *) echo "Unknown arg: $1"; exit 1 ;;
    esac
done

mkdir -p "$REPORT_DIR"

echo -e "${CYAN}"
echo "╔══════════════════════════════════════════════╗"
echo "║     Sim2Sim Navigation Validation Pipeline  ║"
echo "╚══════════════════════════════════════════════╝"
echo -e "${NC}"

# ============================================================
# Phase 1: Pre-check & Build
# ============================================================
echo -e "${BOLD}[Phase 1] Pre-check & Build${NC}"

need_mc_build=false
need_nav_build=false

if [ ! -f "${BUILD_DIR}/aimrt_main" ]; then
    need_mc_build=true
fi
if [ ! -f "${NAV_DIR}/install/setup.bash" ]; then
    need_nav_build=true
fi

if [ "$DO_BUILD" = true ]; then
    if [ "$need_mc_build" = true ]; then
        echo -e "${YELLOW}  → Building motion_control (aimrt_main missing)...${NC}"
        cd "$ROOT_DIR"
        bash scripts/build.sh
    else
        echo -e "${GREEN}  ✓ motion_control already built${NC}"
    fi

    if [ "$need_nav_build" = true ]; then
        echo -e "${YELLOW}  → Building navigation (install/ missing)...${NC}"
        cd "$ROOT_DIR"
        bash scripts/build_nav.sh
    else
        echo -e "${GREEN}  ✓ navigation already built${NC}"
    fi
else
    if [ "$need_mc_build" = true ] || [ "$need_nav_build" = true ]; then
        echo -e "${RED}[ERROR] --no-build but build artifacts missing:${NC}"
        [ "$need_mc_build" = true ] && echo "  Missing: ${BUILD_DIR}/aimrt_main"
        [ "$need_nav_build" = true ] && echo "  Missing: ${NAV_DIR}/install/setup.bash"
        exit 1
    fi
    echo -e "${GREEN}  ✓ Build artifacts present (--no-build)${NC}"
fi

# Check nav scene model
MODEL_PATH="${BUILD_DIR}/cfg/sim_module/model/mjcf/xyber_x1_nav.xml"
if [ ! -f "$MODEL_PATH" ]; then
    echo -e "${RED}[ERROR] Scene model missing: $MODEL_PATH${NC}"
    echo -e "  Rebuild motion_control: scripts/build.sh"
    exit 1
fi
echo -e "${GREEN}  ✓ Scene model: ${MODEL_PATH}${NC}"

# ============================================================
# Phase 2: Cleanup existing session & Launch sim stack
# ============================================================
echo -e "\n${BOLD}[Phase 2] Launch Simulation Stack${NC}"

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
    echo -e "${YELLOW}  → Killing existing session: ${SESSION_NAME}${NC}"
    tmux kill-session -t "${SESSION_NAME}"
    sleep 2
fi

echo -e "${YELLOW}  → Starting run_sim_nav.sh (aimrt + lidar + fastlio + nav2 + octomap + leg_odom)...${NC}"
cd "$ROOT_DIR"
bash scripts/run_sim_nav.sh &
LAUNCH_PID=$!

echo -e "${YELLOW}  → Waiting for sim stack to initialize...${NC}"

# ============================================================
# Phase 3: Wait for readiness
# ============================================================
echo -e "\n${BOLD}[Phase 3] Wait for Readiness${NC}"

# Detect ROS2
if [ -z "${ROS_SETUP_BASH:-}" ]; then
    if [ -f "${CONDA_PREFIX:-}/ros_humble/setup.bash" ]; then
        ROS_SETUP_BASH="${CONDA_PREFIX}/ros_humble/setup.bash"
    elif [ -f /opt/ros/humble/setup.bash ]; then
        ROS_SETUP_BASH="/opt/ros/humble/setup.bash"
    fi
fi
source "${ROS_SETUP_BASH}" 2>/dev/null || true
[ -f "${NAV_DIR}/install/setup.bash" ] && source "${NAV_DIR}/install/setup.bash"

check_topic() {
    ros2 topic list 2>/dev/null | grep -q "$1"
}

check_action() {
    ros2 action list 2>/dev/null | grep -q "$1"
}

READY=false
for i in $(seq 1 $READINESS_TIMEOUT); do
    if check_topic "/mujoco/ground_truth" && check_action "navigate_to_pose"; then
        READY=true
        echo -e "${GREEN}  ✓ All components ready (${i}s)${NC}"
        break
    fi
    # Print progress every 10s
    if [ $((i % 10)) -eq 0 ]; then
        echo -e "${YELLOW}  ... waiting (${i}/${READINESS_TIMEOUT}s)${NC}"
    fi
    sleep 1
done

if [ "$READY" = false ]; then
    echo -e "${RED}[ERROR] Sim stack not ready after ${READINESS_TIMEOUT}s${NC}"
    echo "  Missing topics/actions. Check tmux session:"
    echo "    tmux attach -t ${SESSION_NAME}"
    exit 1
fi

# Extra settle time for Nav2 costmaps
echo -e "${YELLOW}  → Settling 5s for Nav2 costmap initialization...${NC}"
sleep 5

# ============================================================
# Phase 4: Run test scenarios
# ============================================================
echo -e "\n${BOLD}[Phase 4] Run Test Scenarios${NC}"

cd "$ROOT_DIR"

if [ -n "$SINGLE_GOAL" ]; then
    echo -e "${YELLOW}  → Single goal: ${SINGLE_GOAL}${NC}"
    bash scripts/send_nav_goal.sh $SINGLE_GOAL

elif [ -n "$SCENARIOS" ]; then
    echo -e "${YELLOW}  → Selected scenarios: ${SCENARIOS}${NC}"
    # Switch walk_mode first
    bash scripts/send_nav_goal.sh --walk-only
    # Run each scenario
    for s in $(echo "$SCENARIOS" | tr ',' ' '); do
        echo -e "\n${CYAN}  >>> Scenario: ${s}${NC}"
        python3 scripts/nav_test_runner.py --scenario "$s" --report-dir "$REPORT_DIR"
    done

else
    echo -e "${YELLOW}  → Full batch (all scenarios)${NC}"
    bash scripts/send_nav_goal.sh --batch
fi

# ============================================================
# Phase 5: Teardown
# ============================================================
echo -e "\n${BOLD}[Phase 5] Teardown${NC}"

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
    tmux kill-session -t "${SESSION_NAME}"
    echo -e "${GREEN}  ✓ tmux session killed${NC}"
fi

# Kill background launcher
kill $LAUNCH_PID 2>/dev/null || true

# ============================================================
# Phase 6: Summary
# ============================================================
echo -e "\n${BOLD}[Phase 6] Summary${NC}"
echo -e "${GREEN}  Reports saved to: ${REPORT_DIR}${NC}"
echo ""

# Find latest batch summary or individual reports
LATEST_JSON=$(ls -t "${REPORT_DIR}"/*.json 2>/dev/null | head -1)
if [ -n "$LATEST_JSON" ]; then
    echo -e "${CYAN}  Latest result: ${LATEST_JSON}${NC}"
    # Try to show a quick summary
    python3 -c "
import json, sys
with open('${LATEST_JSON}') as f:
    data = json.load(f)
if isinstance(data, list):
    print(f'  Scenarios: {len(data)}')
    for r in data:
        m = r.get('metrics', {})
        name = r['scenario']
        succ = '✅' if m.get('success') else '❌'
        print(f'    {name}: {succ}  drift={m.get(\"drift_mean_m\",\"?\")}m  vmax={m.get(\"vmax_m_s\",\"?\")}m/s')
else:
    m = data.get('metrics', {})
    print(f'  Scenario: {data.get(\"scenario\",\"?\")}')
    print(f'    success={m.get(\"success\")}  drift={m.get(\"drift_mean_m\",\"?\")}m  plan_time={m.get(\"plan_time_s\",\"?\")}s')
" 2>/dev/null || echo -e "${YELLOW}  (see full report in reports/)${NC}"
fi

echo ""
echo -e "${GREEN}╔══════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║          Sim2Sim Test Complete              ║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════════╝${NC}"
