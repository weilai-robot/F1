#!/bin/bash
# ============================================================
# 一键导航: 切 walk_mode + 发导航目标 — send_nav_goal.sh
#
# 前提: run_sim_nav.sh 已启动 (aimrt + lidar + fastlio + nav2 都在跑)
#
# 用法:
#   ./send_nav_goal.sh 5.0 0.0            # 走到 (5.0, 0.0)
#   ./send_nav_goal.sh 5.0 0.0 90         # 到 (5.0,0.0) 且终点朝向 90°
#   ./send_nav_goal.sh 5.0 0.0 0 120      # 超时改成 120s
#   ./send_nav_goal.sh --walk-only        # 只切 walk_mode, 不发目标
#   ./send_nav_goal.sh --batch            # 切 walk_mode 后跑 nav_test_runner 批量场景
#
# 做了什么:
#   1. source ROS2 + navigation workspace
#   2. publish /walk_mode (std_msgs/Float32) 让机器人进入行走 (初始是 stand, 不执行 cmd_vel)
#   3. 调 nav_test_runner.py 发 NavigateToPose 目标并出指标报告
# ============================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
NAV_DIR="${SCRIPT_DIR}/navigation"

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'

# ── 自动检测 ROS2 setup.bash (与 run_sim_nav.sh 一致) ──
if [ -z "${ROS_SETUP_BASH}" ]; then
    if   [ -f "${CONDA_PREFIX:-}/ros_humble/setup.bash" ]; then ROS_SETUP_BASH="${CONDA_PREFIX}/ros_humble/setup.bash"
    elif [ -f "${CONDA_PREFIX:-}/setup.bash" ];            then ROS_SETUP_BASH="${CONDA_PREFIX}/setup.bash"
    elif [ -f /opt/ros/humble/setup.bash ];               then ROS_SETUP_BASH="/opt/ros/humble/setup.bash"
    elif [ -n "${AMENT_PREFIX_PATH:-}" ]; then
        ROS_BASE=$(echo "${AMENT_PREFIX_PATH}" | tr ':' '\n' | tail -1)
        [ -f "${ROS_BASE}/setup.bash" ] && ROS_SETUP_BASH="${ROS_BASE}/setup.bash"
    fi
fi
if [ -z "${ROS_SETUP_BASH}" ] || [ ! -f "${ROS_SETUP_BASH}" ]; then
    echo -e "${RED}[ERROR] 未找到 ROS2 setup.bash, 请 export ROS_SETUP_BASH=/path/to/setup.bash${NC}"
    exit 1
fi
source "${ROS_SETUP_BASH}"
[ -f "${NAV_DIR}/install/setup.bash" ] && source "${NAV_DIR}/install/setup.bash"

# ── 切 walk_mode ──
# 初始状态是 stand, 只会原地站立、忽略 /cmd_vel_limiter; 触发 /walk_mode 进入行走。
# 注意: 不能用 `--once`! 它发完立刻销毁 publisher, ROS2 discovery 还没完成消息就丢了。
# 这里用常驻 publisher 持续发 ~2s, 确保控制模块订上并收到 (控制端有 1s 节流, 多发无害)。
switch_walk_mode() {
    echo -e "${GREEN}[1/2] 切换 walk_mode (机器人应开始原地踏步)...${NC}"
    ros2 topic pub -r 5 /walk_mode std_msgs/msg/Float32 "data: 0.0" >/dev/null 2>&1 &
    local pub_pid=$!
    sleep 2
    kill "${pub_pid}" 2>/dev/null || true
    wait "${pub_pid}" 2>/dev/null || true
    echo -e "${YELLOW}      等待行走控制器稳定 (2s)...${NC}"
    sleep 2
}

# ── 参数解析 ──
case "${1:-}" in
    --walk-only)
        switch_walk_mode
        echo -e "${GREEN}完成: 已进入 walk_mode。可手动发目标或重复运行本脚本。${NC}"
        exit 0
        ;;
    --batch)
        switch_walk_mode
        echo -e "${GREEN}[2/2] 跑 nav_test_runner 批量场景...${NC}"
        exec python3 "${SCRIPT_DIR}/nav_test_runner.py" --batch
        ;;
    "" )
        echo -e "${RED}[ERROR] 缺少目标坐标${NC}"
        echo "用法: ./send_nav_goal.sh <x> <y> [yaw度] [超时s]"
        echo "      ./send_nav_goal.sh --walk-only | --batch"
        exit 1
        ;;
esac

GOAL_X="$1"
GOAL_Y="$2"
YAW_DEG="${3:-0}"
TIMEOUT="${4:-60}"
# 度 -> 弧度
YAW_RAD=$(python3 -c "import math,sys; print(math.radians(float(sys.argv[1])))" "${YAW_DEG}")

switch_walk_mode

echo -e "${GREEN}[2/2] 发送导航目标: (${GOAL_X}, ${GOAL_Y}, yaw=${YAW_DEG}°) timeout=${TIMEOUT}s${NC}"
exec python3 "${SCRIPT_DIR}/nav_test_runner.py" \
    --goal-x "${GOAL_X}" --goal-y "${GOAL_Y}" --goal-yaw "${YAW_RAD}" --timeout "${TIMEOUT}"
