#!/bin/bash
# ============================================================
# 仿真导航一键启动 (sim_module 链路) — run_sim_nav.sh
#
# 全部在 tmux 中启动, 无需额外终端:
#   [0] aimrt_main       — ONNX RL + sim_module 物理仿真 (真机一致)
#   [1] lidar_bridge     — MuJoCo LiDAR 射线追踪 + /clock
#   [2] fast_lio2        — SLAM 里程计
#   [3] nav2             — 导航栈 (AMCL + MPPI + Costmap)
#   [4] octomap          — 3D 地图 (可选)
# ============================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
NAV_DIR="${SCRIPT_DIR}/navigation"
BUILD_DIR="${SCRIPT_DIR}/build"
SESSION_NAME="f1_sim_nav"

# --- 颜色 ---
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}[run_sim_nav] 启动 sim_module 导航链路...${NC}"

# ── 自动检测 ROS2 setup.bash 路径 ──────────────────────────
# 优先级: $ROS_SETUP_BASH > 当前 conda 环境 > /opt/ros/humble > AMENT_PREFIX_PATH
if [ -z "${ROS_SETUP_BASH}" ]; then
    if [ -f "${CONDA_PREFIX:-}/ros_humble/setup.bash" ]; then
        ROS_SETUP_BASH="${CONDA_PREFIX}/ros_humble/setup.bash"
    elif [ -f "${CONDA_PREFIX:-}/setup.bash" ]; then
        ROS_SETUP_BASH="${CONDA_PREFIX}/setup.bash"
    elif [ -f /opt/ros/humble/setup.bash ]; then
        ROS_SETUP_BASH="/opt/ros/humble/setup.bash"
    elif [ -n "${AMENT_PREFIX_PATH:-}" ]; then
        # 从 AMENT_PREFIX_PATH 推断 (最后一个路径通常是 ros2 base)
        ROS_BASE=$(echo "${AMENT_PREFIX_PATH}" | tr ':' '\n' | tail -1)
        if [ -f "${ROS_BASE}/setup.bash" ]; then
            ROS_SETUP_BASH="${ROS_BASE}/setup.bash"
        fi
    fi
fi

if [ -z "${ROS_SETUP_BASH}" ] || [ ! -f "${ROS_SETUP_BASH}" ]; then
    echo -e "${RED}[ERROR] 未找到 ROS2 setup.bash${NC}"
    echo -e "  尝试过的路径:"
    echo -e "    /opt/ros/humble/setup.bash"
    echo -e "    \${CONDA_PREFIX}/setup.bash"
    echo -e "    \${CONDA_PREFIX}/ros_humble/setup.bash"
    echo -e "  请手动设置: export ROS_SETUP_BASH=/path/to/setup.bash"
    exit 1
fi

echo -e "${GREEN}  ROS2: ${ROS_SETUP_BASH}${NC}"

# --- source ---
source "${ROS_SETUP_BASH}"
source "${NAV_DIR}/install/setup.bash"

ros_pkg_exists() {
    ros2 pkg prefix "$1" >/dev/null 2>&1
}

print_missing_ros_pkg_help() {
    local pkg="$1"
    echo -e "${RED}[ERROR] 缺少 ROS2 package: ${pkg}${NC}"
    echo -e "  当前 ROS2: ${ROS_SETUP_BASH}"
    echo -e "  如果使用 apt ROS Humble:"
    echo -e "    sudo apt install ros-humble-nav2-bringup ros-humble-octomap-server"
    echo -e "  如果使用 conda/robostack ROS Humble:"
    echo -e "    mamba install -c robostack-staging -c conda-forge ros-humble-nav2-bringup ros-humble-octomap-server"
}

for pkg in fast_lio humanoid_sim; do
    if ! ros_pkg_exists "${pkg}"; then
        print_missing_ros_pkg_help "${pkg}"
        echo -e "  请先运行: ./build_nav.sh"
        exit 1
    fi
done

HUMANOID_SIM_PREFIX="$(ros2 pkg prefix humanoid_sim)"
ODOM_BRIDGE_EXE="${HUMANOID_SIM_PREFIX}/lib/humanoid_sim/odom_bridge.py"
if [ ! -x "${ODOM_BRIDGE_EXE}" ]; then
    echo -e "${RED}[ERROR] odom_bridge.py 未安装或不可执行: ${ODOM_BRIDGE_EXE}${NC}"
    echo -e "  请重新构建 navigation:"
    echo -e "    ./build_nav.sh"
    exit 1
fi

if ! ros_pkg_exists nav2_bringup; then
    print_missing_ros_pkg_help "nav2_bringup"
    exit 1
fi

ENABLE_OCTOMAP=true
if ! ros_pkg_exists octomap_server; then
    ENABLE_OCTOMAP=false
    echo -e "${YELLOW}[WARN] 缺少 octomap_server，将跳过 [4] OctoMap 窗口。${NC}"
    echo -e "       安装后可恢复: sudo apt install ros-humble-octomap-server"
fi

# --- 检查构建产物 ---
if [ ! -f "${BUILD_DIR}/aimrt_main" ]; then
    echo -e "${RED}[ERROR] motion_control 未构建: ${BUILD_DIR}/aimrt_main 不存在${NC}"
    echo -e "  请先运行: ./build.sh"
    exit 1
fi
if [ ! -f "${NAV_DIR}/install/setup.bash" ]; then
    echo -e "${RED}[ERROR] navigation workspace 未构建${NC}"
    echo -e "  请先运行: ./build_nav.sh"
    exit 1
fi

# --- 检查导航场景模型 ---
MODEL_PATH="${BUILD_DIR}/cfg/sim_module/model/mjcf/xyber_x1_nav.xml"
if [ ! -f "$MODEL_PATH" ]; then
    echo -e "${RED}[ERROR] 未找到场景模型: $MODEL_PATH${NC}"
    echo -e "  请重新运行 ./build.sh (需要 sim_x1_nav.yaml + xyber_x1_nav.xml + lab_env.xml)"
    exit 1
fi

# --- 检查是否已有同名 session ---
if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
    echo -e "${YELLOW}[WARN] 已存在 tmux session: ${SESSION_NAME}${NC}"
    echo -e "  关闭: tmux kill-session -t ${SESSION_NAME}"
    exit 1
fi

# --- 每个窗口的 source 前缀 ---
TMUX_SOURCE="source ${ROS_SETUP_BASH} && source ${NAV_DIR}/install/setup.bash"

# --- [窗口 0] aimrt_main (运动控制 + 物理仿真) ---
tmux new-session -d -s "${SESSION_NAME}" -n "aimrt"
echo -e "${GREEN}  [0] aimrt_main (sim_x1_nav.yaml)${NC}"
tmux send-keys -t "${SESSION_NAME}:0" \
    "cd ${BUILD_DIR} && source install/share/ros2_plugin_proto/local_setup.bash 2>/dev/null; ./aimrt_main --cfg_file_path=./cfg/x1_cfg_sim_nav.yaml" Enter

echo -e "${YELLOW}  等待 aimrt_main + MuJoCo 渲染窗口启动 (5s)...${NC}"
sleep 5

# --- [窗口 1] MuJoCo LiDAR Bridge ---
tmux new-window -t "${SESSION_NAME}" -n "lidar_bridge"
echo -e "${GREEN}  [1] MuJoCo LiDAR Bridge${NC}"
tmux send-keys -t "${SESSION_NAME}:1" "${TMUX_SOURCE}" Enter
tmux send-keys -t "${SESSION_NAME}:1" \
    "export MUJOCO_LIDAR_SRC=${NAV_DIR}/MuJoCo-LiDAR/src" Enter
tmux send-keys -t "${SESSION_NAME}:1" \
    "python3 ${NAV_DIR}/humanoid_sim/scripts/mujoco_lidar_bridge.py --ros-args -p model_path:='${MODEL_PATH}' -p output_type:=pointcloud2 -p downsample:=1 -p lidar_hz:=10" Enter

sleep 2

# --- [窗口 2] FastLIO2 ---
tmux new-window -t "${SESSION_NAME}" -n "fastlio"
echo -e "${GREEN}  [2] FastLIO2 (sim_module_mid360.yaml)${NC}"
tmux send-keys -t "${SESSION_NAME}:2" "${TMUX_SOURCE}" Enter
tmux send-keys -t "${SESSION_NAME}:2" \
    "ros2 launch fast_lio mapping_sim_module.launch.py" Enter

sleep 2

# --- [窗口 3] Nav2 ---
tmux new-window -t "${SESSION_NAME}" -n "nav2"
echo -e "${GREEN}  [3] Nav2 导航${NC}"
tmux send-keys -t "${SESSION_NAME}:3" "${TMUX_SOURCE}" Enter
tmux send-keys -t "${SESSION_NAME}:3" \
    "ros2 launch humanoid_sim navigation.launch.py" Enter

sleep 2

# --- [窗口 4] OctoMap ---
if [ "${ENABLE_OCTOMAP}" = true ]; then
    tmux new-window -t "${SESSION_NAME}" -n "octomap"
    echo -e "${GREEN}  [4] OctoMap 3D 地图${NC}"
    tmux send-keys -t "${SESSION_NAME}:4" "${TMUX_SOURCE}" Enter
    tmux send-keys -t "${SESSION_NAME}:4" \
        "ros2 launch humanoid_sim octomap_mapping.launch.py" Enter
fi

# --- [窗口 5] 测试数据采集 (rosbag + pidstat) ---
tmux new-window -t "${SESSION_NAME}" -n "record"
if [ "${ENABLE_OCTOMAP}" = true ]; then
    RECORD_WINDOW=5
    echo -e "${GREEN}  [5] 测试数据采集 (手动启动)${NC}"
else
    RECORD_WINDOW=4
    echo -e "${GREEN}  [4] 测试数据采集 (手动启动)${NC}"
fi
tmux send-keys -t "${SESSION_NAME}:${RECORD_WINDOW}" "source ${ROS_SETUP_BASH}" Enter
tmux send-keys -t "${SESSION_NAME}:${RECORD_WINDOW}" \
    "echo '=== 导航测试数据采集 ===\n  录制 bag: ros2 bag record /mujoco/ground_truth /cmd_vel_limiter /Odometry /tf -o test_run_NNN\n  CPU/内存: pidstat -ru 1 -C \"aimrt_main|mujoco_lidar_bridge|fastlio|nav2\" > cpu_mem.log\n  实时监控: ros2 topic echo /mujoco/ground_truth --once'" Enter

# --- 完成 ---
echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN} sim_module 导航链路已启动${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo " tmux 窗口布局:"
echo "   [0] aimrt        - aimrt_main (ONNX RL + MuJoCo 物理)"
echo "   [1] lidar_bridge - MuJoCo LiDAR 射线追踪 + /clock"
echo "   [2] fastlio      - FastLIO2 里程计"
echo "   [3] nav2         - Nav2 导航栈"
if [ "${ENABLE_OCTOMAP}" = true ]; then
    echo "   [4] octomap      - OctoMap 3D 地图"
    echo "   [5] record       - 测试数据采集 (rosbag + pidstat)"
else
    echo "   [4] record       - 测试数据采集 (rosbag + pidstat)"
fi
echo ""
echo " 切换窗口: Ctrl+B 然后 数字键"
echo " 附加终端: tmux attach -t ${SESSION_NAME}"
echo " 关闭全部: tmux kill-session -t ${SESSION_NAME}"
echo ""
echo -e "${YELLOW} 操作流程:${NC}"
echo -e "   1. 在 MuJoCo 窗口中确认机器人已加载"
echo -e "   2. 手柄按 [stand_mode] → [walk_mode] 进入行走"
echo -e "   3. 用 RViz 或 ros2 action 发送导航目标"
