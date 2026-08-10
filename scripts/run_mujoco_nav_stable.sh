#!/bin/bash
# ============================================================
# 仿真导航一键启动 (MuJoCo + sim_module 联合链路) — run_mujoco_nav.sh
# 注意: 本路线联合 aimrt_main/sim_module (MuJoCo 物理 + ONNX RL 控制), 与真机一致。
#       若需 Gazebo 路线 (不联合 sim_module), 请用 run_gazebo_nav.sh
#
# 全部在 tmux 中启动, 无需额外终端:
#   [0] aimrt_main       — ONNX RL + sim_module 物理仿真 (真机一致)
#   [1] lidar_bridge     — MuJoCo LiDAR 射线追踪 + /clock
#   [2] imu              — /livox/imu → /livox/imu_200 转发 (FastLIO)
#   [3] fast_lio2        — SLAM 里程计
#   [3] open3d_loc       — ICP 全局定位 (发布 map->odom / map->camera_init TF)
#   [4] nav2             — 导航栈 (MPPI + Costmap)
#   [5] octomap          — 3D 地图 (可选)
# ============================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
NAV_DIR="${ROOT_DIR}/navigation"
BUILD_DIR="${ROOT_DIR}/build"
SESSION_NAME="f1_sim_nav"

# --- 颜色 ---
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

WAIT_CLOCK_TIMEOUT=20
WAIT_IMU_TIMEOUT=20
WAIT_FASTLIO_TIMEOUT=30
WAIT_MAP_TIMEOUT=30
WAIT_NAV2_TIMEOUT=30
SESSION_CREATED=0

launch_cleanup() {
    local rc=$?
    if [ "${SESSION_CREATED}" = 1 ] && [ "${rc}" -ne 0 ]; then
        echo ""
        echo -e "${RED}[FAIL] 启动中断 (rc=${rc}).${NC}" >&2
        echo -e "${YELLOW}  tmux session ${SESSION_NAME} 可能仍在运行，可先排查现场:${NC}" >&2
        echo -e "${YELLOW}    tmux attach -t ${SESSION_NAME}${NC}" >&2
        echo -e "${YELLOW}  排查完后清理:${NC}" >&2
        echo -e "${YELLOW}    tmux kill-session -t ${SESSION_NAME}${NC}" >&2
    fi
}
trap launch_cleanup EXIT

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

for pkg in fast_lio humanoid_sim open3d_loc; do
    if ! ros_pkg_exists "${pkg}"; then
        print_missing_ros_pkg_help "${pkg}"
        echo -e "  请先运行: ./build_nav.sh"
        exit 1
    fi
done

# --- CustomMsg 依赖检查 (output_type:=custom 需要 livox_ros_driver2) ---
if ! python3 -c "from livox_ros_driver2.msg import CustomMsg, CustomPoint" >/dev/null 2>&1; then
    echo -e "${RED}[ERROR] 未检测到 livox_ros_driver2 Python 消息: livox_ros_driver2.msg.CustomMsg${NC}"
    echo -e "  当前配置会启动 output_type:=custom，因此必须可导入 livox_ros_driver2。"
    echo -e "  请重新构建 navigation（不要跳过 livox）:"
    echo -e "    ./build_nav.sh"
    exit 1
fi

# --- FastLIO2 配置文件检查 (必须安装到 share/fast_lio/config) ---
FAST_LIO_PREFIX="$(ros2 pkg prefix fast_lio)"
FAST_LIO_CFG="${FAST_LIO_PREFIX}/share/fast_lio/config/sim_module_mid360_custom.yaml"
if [ ! -f "${FAST_LIO_CFG}" ]; then
    echo -e "${RED}[ERROR] FastLIO2 配置未安装: ${FAST_LIO_CFG}${NC}"
    echo -e "  请重新构建 fast_lio（确保 config 被 install）:"
    echo -e "    ./build_nav.sh --packages-select fast_lio"
    exit 1
fi

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
    echo -e "${YELLOW}[WARN] 缺少 octomap_server，将跳过 [5] OctoMap 窗口。${NC}"
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
# TMUX_SOURCE="source ${ROS_SETUP_BASH} && source ${NAV_DIR}/install/setup.bash"
TMUX_SOURCE="source /home/robot/env/miniconda/etc/profile.d/conda.sh && conda deactivate >/dev/null 2>&1 || true && conda activate nav && source ${ROS_SETUP_BASH} && source ${NAV_DIR}/install/setup.bash"

wait_for_topic_once() {
    local topic="$1"
    local timeout_sec="$2"
    local label="$3"
    local end_ts=$((SECONDS + timeout_sec))

    echo -e "${YELLOW}  等待 ${label} (${topic}, timeout=${timeout_sec}s)...${NC}"
    while (( SECONDS < end_ts )); do
        if timeout 2s ros2 topic echo "${topic}" --once >/dev/null 2>&1; then
            echo -e "${GREEN}  ✓ ${label} 已就绪${NC}"
            return 0
        fi
        sleep 1
    done

    echo -e "${RED}[ERROR] 等待 ${label} 超时: ${topic}${NC}"
    return 1
}

# --- [窗口 0] aimrt_main (运动控制 + 物理仿真) ---
tmux new-session -d -s "${SESSION_NAME}" -n "aimrt"
SESSION_CREATED=1
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
    "export MUJOCO_LIDAR_SRC=${NAV_DIR}/sim/MuJoCo-LiDAR/src" Enter
tmux send-keys -t "${SESSION_NAME}:1" \
        "python3 ${NAV_DIR}/planning/humanoid_sim/scripts/mujoco_lidar_bridge.py --ros-args -p model_path:='${MODEL_PATH}' -p output_type:=custom -p downsample:=5 -p lidar_hz:=10" Enter

wait_for_topic_once "/clock" "${WAIT_CLOCK_TIMEOUT}" "仿真时钟"

# --- [窗口 2] livox IMU 200Hz 转发 ---
tmux new-window -t "${SESSION_NAME}" -n "imu"
echo -e "${GREEN}  [2] livox_imu_throttle (/livox/imu -> /livox/imu_200)${NC}"
tmux send-keys -t "${SESSION_NAME}:imu" "${TMUX_SOURCE}" Enter
tmux send-keys -t "${SESSION_NAME}:imu" \
    "ros2 run humanoid_sim livox_imu_throttle.py" Enter

wait_for_topic_once "/livox/imu_200" "${WAIT_IMU_TIMEOUT}" "200Hz IMU 转发"

# --- [窗口 3] FastLIO2 ---
tmux new-window -t "${SESSION_NAME}" -n "fastlio"
echo -e "${GREEN}  [3] FastLIO2 (sim_module_mid360_custom.yaml, CustomMsg)${NC}"
tmux send-keys -t "${SESSION_NAME}:3" "${TMUX_SOURCE}" Enter
tmux send-keys -t "${SESSION_NAME}:3" \
    "ros2 launch fast_lio mapping_sim_module.launch.py config_file:=sim_module_mid360_custom.yaml" Enter

wait_for_topic_once "/Odometry" "${WAIT_FASTLIO_TIMEOUT}" "FastLIO 里程计"
wait_for_topic_once "/cloud_registered_body" "${WAIT_FASTLIO_TIMEOUT}" "FastLIO 体节点云"

# --- [窗口 4] open3d_loc (ICP) ---
tmux new-window -t "${SESSION_NAME}" -n "icp"
echo -e "${GREEN}  [4] open3d_loc (ICP)${NC}"
tmux send-keys -t "${SESSION_NAME}:4" "${TMUX_SOURCE}" Enter
tmux send-keys -t "${SESSION_NAME}:4" \
    "ros2 launch open3d_loc open3d_loc_x1.launch.py use_sim_time:=true" Enter

wait_for_topic_once "/odom2map" "${WAIT_FASTLIO_TIMEOUT}" "Open3D odom2map 输出"

# --- [窗口 5] Nav2 ---
tmux new-window -t "${SESSION_NAME}" -n "nav2"
echo -e "${GREEN}  [5] Nav2 导航${NC}"
tmux send-keys -t "${SESSION_NAME}:5" "${TMUX_SOURCE}" Enter
tmux send-keys -t "${SESSION_NAME}:5" \
    "ros2 launch humanoid_sim navigation.launch.py" Enter

wait_for_topic_once "/map" "${WAIT_MAP_TIMEOUT}" "静态地图"
echo -e "${YELLOW}  等待 Nav2 action server (${WAIT_NAV2_TIMEOUT}s)...${NC}"
if timeout "${WAIT_NAV2_TIMEOUT}s" ros2 action info /navigate_to_pose >/dev/null 2>&1; then
    echo -e "${GREEN}  ✓ Nav2 action server 已就绪${NC}"
else
    echo -e "${RED}[ERROR] 等待 Nav2 action server 超时${NC}"
    exit 1
fi

# --- [窗口 6] OctoMap --- (临时注释：当前不需要)
# if [ "${ENABLE_OCTOMAP}" = true ]; then
#     tmux new-window -t "${SESSION_NAME}" -n "octomap"
#     echo -e "${GREEN}  [6] OctoMap 3D 地图${NC}"
#     tmux send-keys -t "${SESSION_NAME}:6" "${TMUX_SOURCE}" Enter
#     tmux send-keys -t "${SESSION_NAME}:6" \
#         "ros2 launch humanoid_sim octomap_mapping.launch.py" Enter
# fi

# --- [窗口 7] 腿里程计 (Leg Odometry 前馈) --- (临时注释：当前不需要)
# tmux new-window -t "${SESSION_NAME}" -n "leg_odom"
# echo -e "${GREEN}  [7] 腿里程计 (Leg Odometry)${NC}"
# tmux send-keys -t "${SESSION_NAME}:leg_odom" "${TMUX_SOURCE}" Enter
# tmux send-keys -t "${SESSION_NAME}:leg_odom" \
#     "ros2 run humanoid_sim leg_odom_node.py --ros-args -p model_path:='${MODEL_PATH}'" Enter

# --- [窗口 8] 测试数据采集 (rosbag + pidstat) ---
tmux new-window -t "${SESSION_NAME}" -n "record"
echo -e "${GREEN}  [8] 测试数据采集 (手动启动)${NC}"
tmux send-keys -t "${SESSION_NAME}:record" "source ${ROS_SETUP_BASH}" Enter
tmux send-keys -t "${SESSION_NAME}:record" \
    "echo '=== 导航测试数据采集 ===\n  录制 bag: ros2 bag record /mujoco/ground_truth /cmd_vel /cmd_vel_limiter /Odometry /leg_odom /tf -o test_run_NNN\n  CPU/内存: pidstat -ru 1 -C \"aimrt_main|mujoco_lidar_bridge|fastlio|open3d_loc|nav2\" > cpu_mem.log\n  实时监控: ros2 topic echo /mujoco/ground_truth --once'" Enter

# --- 完成 ---
echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN} sim_module 导航链路已启动${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo " tmux 窗口布局:"
echo "   [0] aimrt        - aimrt_main (ONNX RL + MuJoCo 物理)"
echo "   [1] lidar_bridge - MuJoCo LiDAR 射线追踪 + /clock"
echo "   [2] imu          - /livox/imu -> /livox/imu_200 (200Hz)"
echo "   [3] fastlio      - FastLIO2 里程计"
echo "   [4] icp          - open3d_loc (ICP 全局定位)"
echo "   [5] nav2         - Nav2 导航栈"
echo "   octomap         - (已注释/当前不启用)"
echo "   leg_odom        - (已注释/当前不启用)"
echo "   record          - 测试数据采集 (rosbag + pidstat)（窗口index随配置变化）"
echo ""
echo " 切换窗口: Ctrl+B 然后 数字键"
echo " 附加终端: tmux attach -t ${SESSION_NAME}"
echo " 关闭全部: tmux kill-session -t ${SESSION_NAME}"
echo ""
echo -e "${YELLOW} 操作流程:${NC}"
echo -e "   1. 在 MuJoCo 窗口中确认机器人已加载"
echo -e "   2. 手柄按 [stand_mode] → [walk_mode] 进入行走"
echo -e "   3. 用 RViz 或 ros2 action 发送导航目标"
