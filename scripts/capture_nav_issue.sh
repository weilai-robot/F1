#!/bin/bash
# ============================================================
#  capture_nav_issue.sh — 手动抓取导航异常现场
#
#  用法:
#    1. 先正常运行 run_mujoco_nav.sh
#    2. 出现 "costmap 只剩一小块 / goal 没反应" 时，立刻执行:
#         ./capture_nav_issue.sh
#    3. 脚本会把 TF / map / lifecycle / topic 频率 / tmux 日志保存到:
#         F1/reports/nav_issue_capture/<timestamp>/
#
#  设计原则:
#    - 平时不常驻，不额外占资源
#    - 只有故障出现时手动抓一次
#    - 允许部分命令失败，但尽量把现场留全
# ============================================================
set -u

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
NAV_DIR="${ROOT_DIR}/navigation"
REPORT_ROOT="${ROOT_DIR}/reports/nav_issue_capture"
SESSION_NAME="${SESSION_NAME:-f1_sim_nav}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${REPORT_ROOT}/${STAMP}"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

mkdir -p "${OUT_DIR}"

echo -e "${GREEN}[capture] 输出目录:${NC} ${OUT_DIR}"

# shellcheck source=/dev/null
source "${SCRIPT_DIR}/ros2_source.sh"
[ -f "${NAV_DIR}/install/setup.bash" ] && source "${NAV_DIR}/install/setup.bash"

run_capture() {
    local name="$1"
    shift
    local outfile="${OUT_DIR}/${name}.txt"
    echo -e "${YELLOW}[capture]${NC} ${name}"
    {
        echo "# command: $*"
        echo "# time: $(date --iso-8601=seconds)"
        echo
        "$@"
    } >"${outfile}" 2>&1 || true
}

run_capture_shell() {
    local name="$1"
    local cmd="$2"
    local outfile="${OUT_DIR}/${name}.txt"
    echo -e "${YELLOW}[capture]${NC} ${name}"
    {
        echo "# command: ${cmd}"
        echo "# time: $(date --iso-8601=seconds)"
        echo
        bash -lc "${cmd}"
    } >"${outfile}" 2>&1 || true
}

capture_tmux_window() {
    local window="$1"
    local outfile="${OUT_DIR}/tmux_${window}.log"
    if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
        tmux capture-pane -pt "${SESSION_NAME}:${window}" -S -300 >"${outfile}" 2>&1 || true
    else
        echo "tmux session not found: ${SESSION_NAME}" >"${outfile}"
    fi
}

echo -e "${YELLOW}[capture]${NC} 基础 ROS 状态"
run_capture "node_list" ros2 node list
run_capture "topic_list" ros2 topic list -t
run_capture "action_navigate_to_pose" ros2 action info /navigate_to_pose
run_capture "topic_info_tf" ros2 topic info /tf -v
run_capture "topic_info_map" ros2 topic info /map -v
run_capture "topic_info_odom2map" ros2 topic info /odom2map -v

echo -e "${YELLOW}[capture]${NC} 一次性消息快照"
run_capture_shell "echo_map_once" "timeout 5s ros2 topic echo /map --once"
run_capture_shell "echo_odom2map_once" "timeout 5s ros2 topic echo /odom2map --once"
run_capture_shell "echo_localization_confidence_once" "timeout 5s ros2 topic echo /localization_3d_confidence --once"
run_capture_shell "tf2_echo_map_odom" "timeout 5s ros2 run tf2_ros tf2_echo map odom"
run_capture_shell "tf2_echo_map_camera_init" "timeout 5s ros2 run tf2_ros tf2_echo map camera_init"
run_capture_shell "tf2_echo_odom_base_footprint" "timeout 5s ros2 run tf2_ros tf2_echo odom base_footprint"
run_capture_shell "tf2_echo_base_link_lidar_link" "timeout 5s ros2 run tf2_ros tf2_echo base_link lidar_link"

echo -e "${YELLOW}[capture]${NC} 生命周期"
for node in /map_server /planner_server /controller_server /bt_navigator /recoveries_server /waypoint_follower /behavior_server; do
    run_capture_shell "lifecycle_${node//\//_}" "timeout 5s ros2 lifecycle get ${node}"
done

echo -e "${YELLOW}[capture]${NC} 关键 topic 频率"
run_capture_shell "hz_clock" "timeout 8s ros2 topic hz /clock"
run_capture_shell "hz_livox_lidar" "timeout 8s ros2 topic hz /livox/lidar"
run_capture_shell "hz_odometry" "timeout 8s ros2 topic hz /Odometry"
run_capture_shell "hz_cloud_registered_body" "timeout 8s ros2 topic hz /cloud_registered_body"
run_capture_shell "hz_cmd_vel_limiter" "timeout 8s ros2 topic hz /cmd_vel_limiter"

echo -e "${YELLOW}[capture]${NC} TF 图"
(
    cd "${OUT_DIR}" || exit 1
    timeout 10s ros2 run tf2_tools view_frames >view_frames.stdout 2>view_frames.stderr || true
)

echo -e "${YELLOW}[capture]${NC} tmux 日志快照"
run_capture "tmux_sessions" tmux ls
run_capture_shell "tmux_windows" "tmux list-windows -t ${SESSION_NAME}"
capture_tmux_window "aimrt"
capture_tmux_window "lidar_bridge"
capture_tmux_window "imu"
capture_tmux_window "fastlio"
capture_tmux_window "icp"
capture_tmux_window "nav2"
capture_tmux_window "octomap"
capture_tmux_window "leg_odom"

cat >"${OUT_DIR}/README.txt" <<EOF
capture_nav_issue.sh 现场抓取目录

重点先看:
1. echo_odom2map_once.txt
2. tf2_echo_map_odom.txt
3. tf2_echo_map_camera_init.txt
4. echo_map_once.txt
5. lifecycle__map_server.txt
6. lifecycle__planner_server.txt
7. lifecycle__controller_server.txt
8. hz_clock.txt / hz_odometry.txt / hz_cloud_registered_body.txt
9. tmux_icp.log / tmux_nav2.log / tmux_fastlio.log

如果 frames.pdf / frames.yaml 成功生成，也一起查看。
EOF

echo -e "${GREEN}[capture] 完成${NC}"
echo "  目录: ${OUT_DIR}"
echo "  建议优先看 README.txt 里列出的文件"
