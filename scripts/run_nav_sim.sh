#!/bin/bash
# ============================================================
# 仿真导航一键启动 — run_nav_sim.sh
# 启动内容 (4 个 tmux 窗口):
#   [0] Gazebo 仿真环境 + 机器人模型
#   [1] FastLIO2 里程计 (SLAM)
#   [2] Nav2 导航 (odom_bridge + AMCL + MPPI + Costmap)
#   [3] OctoMap 3D 地图 (可选，用于 VoxelLayer)
#
# 使用方式:
#   ./run_nav_sim.sh          # 完整导航链路
#   ./run_nav_sim.sh --no-gui  # 无头模式 (不弹 RViz/Gazebo GUI)
#
# 前置条件:
#   1. cd navigation && colcon build --symlink-install
#   2. source navigation/install/setup.bash
# ============================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
NAV_DIR="${ROOT_DIR}/navigation"
SESSION_NAME="f1_nav_sim"

# --- 颜色 ---
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}[run_nav_sim] 启动仿真导航栈...${NC}"

# --- 检查 navigation workspace 是否已构建 ---
if [ ! -f "${NAV_DIR}/install/setup.bash" ]; then
    echo -e "${RED}[ERROR] navigation workspace 未构建${NC}"
    echo -e "  请先运行: cd navigation && colcon build --symlink-install"
    exit 1
fi

# --- source ROS2 ---
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
fi

source "${NAV_DIR}/install/setup.bash"

# --- 检查是否已有同名 session ---
if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
    echo -e "${YELLOW}[WARN] 已存在 tmux session: ${SESSION_NAME}${NC}"
    echo -e "  切换查看: tmux attach -t ${SESSION_NAME}"
    echo -e "  关闭后重试: tmux kill-session -t ${SESSION_NAME}"
    exit 1
fi

# --- 解析参数 ---
RVIZ_ARG=""
if [ "$1" = "--no-gui" ]; then
    RVIZ_ARG="rviz:=false"
    echo -e "${YELLOW}[run_nav_sim] 无头模式: 不启动 RViz${NC}"
fi

# --- 创建 tmux session ---
tmux new-session -d -s "${SESSION_NAME}" -n "gazebo"

# --- [窗口 0] Gazebo 仿真环境 ---
echo -e "${GREEN}  [0] Gazebo 仿真环境${NC}"
tmux send-keys -t "${SESSION_NAME}:0" \
    "source /opt/ros/humble/setup.bash && source ${NAV_DIR}/install/setup.bash" Enter
tmux send-keys -t "${SESSION_NAME}:0" \
    "ros2 launch humanoid_sim simulation.launch.py" Enter

sleep 3

# --- [窗口 1] FastLIO2 里程计 ---
tmux new-window -t "${SESSION_NAME}" -n "fastlio"
echo -e "${GREEN}  [1] FastLIO2 里程计${NC}"
tmux send-keys -t "${SESSION_NAME}:1" \
    "source /opt/ros/humble/setup.bash && source ${NAV_DIR}/install/setup.bash" Enter
tmux send-keys -t "${SESSION_NAME}:1" \
    "ros2 launch fast_lio mapping.launch.py ${RVIZ_ARG}" Enter

sleep 2

# --- [窗口 2] Nav2 导航 ---
tmux new-window -t "${SESSION_NAME}" -n "nav2"
echo -e "${GREEN}  [2] Nav2 导航 (tf_bridge + AMCL + MPPI + Costmap)${NC}"
tmux send-keys -t "${SESSION_NAME}:2" \
    "source /opt/ros/humble/setup.bash && source ${NAV_DIR}/install/setup.bash" Enter
tmux send-keys -t "${SESSION_NAME}:2" \
    "ros2 launch humanoid_sim navigation.launch.py" Enter

sleep 2

# --- [窗口 3] OctoMap (可选，3D 障碍物地图) ---
tmux new-window -t "${SESSION_NAME}" -n "octomap"
echo -e "${GREEN}  [3] OctoMap 3D 地图 (VoxelLayer 数据源)${NC}"
tmux send-keys -t "${SESSION_NAME}:3" \
    "source /opt/ros/humble/setup.bash && source ${NAV_DIR}/install/setup.bash" Enter
tmux send-keys -t "${SESSION_NAME}:3" \
    "ros2 launch humanoid_sim octomap_mapping.launch.py" Enter

# --- 完成 ---
echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN} 仿真导航栈已启动${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo " tmux 窗口布局:"
echo "   [0] gazebo   - Gazebo 仿真环境"
echo "   [1] fastlio  - FastLIO2 里程计"
echo "   [2] nav2     - Nav2 导航栈"
echo "   [3] octomap  - OctoMap 3D 地图"
echo ""
echo " 切换窗口: Ctrl+B 然后 数字键 (0-3)"
echo " 附加终端: tmux attach -t ${SESSION_NAME}"
echo " 关闭全部: tmux kill-session -t ${SESSION_NAME}"
echo ""
echo -e "${YELLOW} 提示: 另开终端运行 ./run_sim.sh 启动运动控制${NC}"
