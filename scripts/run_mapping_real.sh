#!/bin/bash
# ============================================================
# 真机建图一键启动 — run_mapping_real.sh（吊架 · 倒装 native）
# 对齐 run_nav_real.sh 结构；SOP: docs/sim2real/真机建图SOP-机器人吊架native-2026-10-26.md
#
# 职责边界（SOP §0）: 现场只做「调平 + 录包 + 现场监控」；
#   权威 3D pcd / 2D pgm 回 WSL 离线重放生成（SOP §4 阶段 3）。
#   现场 fastlio / octomap 均为纯订阅监控（不碰 bag 数据流）。
#
# tmux 窗口:
#   [tilt]    — imu_tilt_check.py 倾角读数（调平 θ<0.3°；结束时复用作「拆架前」读数）
#   [livox]   — Livox MID-360 驱动
#   [fastlio] — FastLIO2 现场监控（F1_mapping_mid360.yaml；pcd 运行时 patch → ~/bags/<MAP>_field.pcd）
#   [octomap] — 倒装 z 窗 live 2D 投影监控（h = 雷达离地高）
#   [record]  — ros2 bag record /livox/lidar /livox/imu → ~/bags/<MAP>
#   切窗（attach 后，默认前缀 Ctrl+B）: Ctrl+B w 列表选窗 · n/p 下/上窗 · 0-9 按序号
#   进窗: tmux attach -t f1_mapping_real · 脱离回终端: Ctrl+B d（窗内进程继续跑）
#
# ⚠ 顺序约束: fastlio 必须在调平后启动——camera_init = 启动时 IMU 系（FAST-LIO 无重力
#   对齐），倾斜状态下启动会把吊挂倾角 β 焊进地图 z 轴（SOP §6.2）。脚本用交互闸门强制。
#
# 用法:
#   ./run_mapping_real.sh                    # MAP=robot_lab_<今日>，h=1.25
#   ./run_mapping_real.sh mylab --h 1.22     # 自定义地图名 + 卷尺实测雷达离地高
#   ./run_mapping_real.sh --no-rviz          # MiniPC 减负（不起 fastlio RViz）
#   ./run_mapping_real.sh --no-octomap       # 不起 octomap 监控
#   ./run_mapping_real.sh --yes              # 跳过调平交互确认（不建议）
#
# 前置:
#   1. ./build_nav.sh（navigation workspace，含 fast_lio / humanoid_sim）
#   2. MID360 联网: ping 192.168.1.195（host eth1=.50，minipc_mid360_eth_static_ip.md）
#   3. 吊架挂好、机器人吊稳；卷尺量好雷达离地 h
#
# 结束顺序（SOP §4 阶段 1）: bag/pcd 的收尾都挂在 Ctrl+C 上——kill-session 只做最后清扫
#   回起点静置 3s → [record] Ctrl+C → [fastlio] Ctrl+C 等 PCD save 打印 →
#   [octomap] Ctrl+C → [tilt] 记「拆架前」读数（三时点对账 §6.3）→ kill-session
# ============================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
NAV_DIR="${ROOT_DIR}/navigation"
SESSION_NAME="f1_mapping_real"
LIDAR_IP="192.168.1.195"
MAPPING_CFG_NAME="F1_mapping_mid360.yaml"

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
MAP=""
H="1.25"
RVIZ_ARG="rviz:=true"
while [ $# -gt 0 ]; do
  case "$1" in
    --h)
      [ $# -ge 2 ] || { echo -e "${RED}[ERROR] --h 缺参数值${NC}"; exit 1; }
      H="$2"; shift 2 ;;
    --yes) YES=1; shift ;;
    --no-rviz) RVIZ_ARG="rviz:=false"; shift ;;
    --no-octomap) NO_OCTOMAP=1; shift ;;
    -h|--help) awk 'NR>1 && /^set -e/{exit} NR>1' "$0"; exit 0 ;;  # 头部注释即帮助
    -*) echo -e "${RED}[ERROR] 未知参数: ${1}${NC}"; exit 1 ;;
    *)
      if [ -z "${MAP}" ]; then MAP="$1"; else
        echo -e "${RED}[ERROR] 多余参数: ${1}（MAP 已设 ${MAP}）${NC}"; exit 1
      fi
      shift ;;
  esac
done
MAP="${MAP:-robot_lab_$(date +%Y%m%d)}"
BAG_DIR="${HOME}/bags"
FIELD_PCD="${BAG_DIR}/${MAP}_field.pcd"

echo -e "${GREEN}[run_mapping_real] 真机建图启动（吊架·倒装 native）${NC}"
echo -e "${GREEN}  MAP=${MAP}  h=${H}m  bag → ${BAG_DIR}/${MAP}/${MAP}_0.db3${NC}"
echo -e "${YELLOW}  现场只录包；权威 3D/2D 产物回 WSL 离线重放生成（SOP §4 阶段 3）${NC}"

# ── ROS2 setup（照抄 run_nav_real.sh：优先 scripts/ros2_source.sh，再回退探测）──
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

ros_pkg_exists() { ros2 pkg prefix "$1" >/dev/null 2>&1; }
for pkg in fast_lio livox_ros_driver2; do
  if ! ros_pkg_exists "${pkg}"; then
    echo -e "${RED}[ERROR] 缺少 ROS2 package: ${pkg}${NC}"
    echo -e "  请先: ./build_nav.sh"
    exit 1
  fi
done

# 建图配置必须已 install（两包均 install(DIRECTORY)，--symlink-install 下新文件即放即生效）
FAST_LIO_PREFIX="$(ros2 pkg prefix fast_lio)"
MAPPING_CFG="${FAST_LIO_PREFIX}/share/fast_lio/config/${MAPPING_CFG_NAME}"
if [ ! -f "${MAPPING_CFG}" ]; then
  echo -e "${RED}[ERROR] 建图配置未安装: ${MAPPING_CFG}${NC}"
  echo -e "  源文件: navigation/localization/fast_lio2/config/${MAPPING_CFG_NAME}"
  echo -e "  请重新构建: ./build_nav.sh --packages-select fast_lio"
  exit 1
fi

# octomap 监控为可选（缺包只降级、不阻塞录包主线）
OCTOMAP_OK=1
ros_pkg_exists octomap_server || OCTOMAP_OK=""
HUMANOID_SIM_PREFIX="$(ros2 pkg prefix humanoid_sim 2>/dev/null || true)"
OCTOMAP_LAUNCH="${HUMANOID_SIM_PREFIX}/share/humanoid_sim/launch/octomap_robot_mapping.launch.py"
[ -f "${OCTOMAP_LAUNCH}" ] || OCTOMAP_OK=""
if [ -z "${NO_OCTOMAP:-}" ] && [ -z "${OCTOMAP_OK}" ]; then
  echo -e "${YELLOW}[WARN] octomap 监控不可用（缺 octomap_server 或 octomap_robot_mapping.launch.py）——降级跳过${NC}"
  echo -e "${YELLOW}       如需: ./build_nav.sh --packages-select humanoid_sim && sudo apt install ros-humble-octomap-server(按实际)${NC}"
  NO_OCTOMAP=1
fi

# 雷达连通性软检查（不通时驱动窗会报错，这里提前提示）
if ! ping -c 1 -W 2 "${LIDAR_IP}" >/dev/null 2>&1; then
  echo -e "${YELLOW}[WARN] MID360 (${LIDAR_IP}) ping 不通——查雷达供电 / eth1 静态 IP .50${NC}"
  echo -e "${YELLOW}       配置见 docs/sim2real/硬件与网络/minipc_mid360_eth_static_ip.md${NC}"
fi

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
  echo -e "${YELLOW}[WARN] 已存在 tmux session: ${SESSION_NAME}${NC}"
  echo -e "  关闭: tmux kill-session -t ${SESSION_NAME}"
  exit 1
fi

mkdir -p "${BAG_DIR}"

# 运行时 patch pcd 落点 → 现场 bag 旁（committed 配置里的 WSL 路径仅离线重放用，MiniPC 上不存在）
RUNTIME_CFG_DIR="$(mktemp -d /tmp/f1_mapping_cfg.XXXXXX)"
sed "s|^\( *\)map_file_path:.*|\1map_file_path: \"${FIELD_PCD}\"|" \
  "${MAPPING_CFG}" > "${RUNTIME_CFG_DIR}/${MAPPING_CFG_NAME}"

if [ -z "${TMUX_SOURCE:-}" ]; then
  TMUX_SOURCE="source ${ROS_SETUP_BASH} && source ${NAV_DIR}/install/setup.bash"
fi

# --- [livox] ---
tmux new-session -d -s "${SESSION_NAME}" -n "livox"
SESSION_CREATED=1
echo -e "${GREEN}  [livox] Livox MID-360 驱动${NC}"
tmux send-keys -t "${SESSION_NAME}:livox" "${TMUX_SOURCE}" Enter
tmux send-keys -t "${SESSION_NAME}:livox" \
  "ros2 launch livox_ros_driver2 msg_MID360_launch.py" Enter
echo -e "${YELLOW}  等待雷达数据 (5s)...${NC}"
sleep 5

# --- [tilt] 调平读数（必须先于 fastlio：camera_init = 启动时 IMU 系）---
tmux new-window -t "${SESSION_NAME}" -n "tilt"
echo -e "${GREEN}  [tilt] imu_tilt_check.py 倾角读数${NC}"
tmux send-keys -t "${SESSION_NAME}:tilt" "${TMUX_SOURCE}" Enter
tmux send-keys -t "${SESSION_NAME}:tilt" "python3 ${SCRIPT_DIR}/imu_tilt_check.py" Enter

# --- 调平交互闸门（θ<0.3° 才放行；--yes 跳过）---
if [ -z "${YES:-}" ]; then
  echo ""
  echo -e "${YELLOW}→ 在 tmux 窗 [tilt] 读数并调平（θ<0.3°）：只动吊带/挂点，勿动 mount 垫片（β/δ 混淆，SOP §6.2）${NC}"
  echo -e "${YELLOW}  进窗看读数: 另开终端（第二个 ssh）跑 tmux attach -t ${SESSION_NAME}，Ctrl+B w 选 [tilt]${NC}"
  echo -e "${YELLOW}  看完 Ctrl+B d 脱离回终端、窗内进程不受影响；本终端留着等下面回车放行${NC}"
  echo -e "${YELLOW}  [tilt] 长时间无读数 → 查 [livox] 窗驱动报错${NC}"
  read -r -p "调平完成后回车，启动 fastlio + octomap + 录包（Ctrl+C 放弃）: " || echo "(非交互终端，直接继续)"
fi

# --- [fastlio] 现场监控（起始静置段完成 IMU init；bag 同段开录，离线重放同口径）---
tmux new-window -t "${SESSION_NAME}" -n "fastlio"
echo -e "${GREEN}  [fastlio] FastLIO2 建图监控（${MAPPING_CFG_NAME}，pcd → ${FIELD_PCD}）${NC}"
tmux send-keys -t "${SESSION_NAME}:fastlio" "${TMUX_SOURCE}" Enter
tmux send-keys -t "${SESSION_NAME}:fastlio" \
  "ros2 launch fast_lio mapping_real.launch.py config_path:=${RUNTIME_CFG_DIR} config_file:=${MAPPING_CFG_NAME} ${RVIZ_ARG}" Enter
sleep 3

# --- [octomap] 倒装 z 窗 2D 投影监控（z 窗 = [h-1.6, h-0.1]）---
if [ -z "${NO_OCTOMAP:-}" ]; then
  tmux new-window -t "${SESSION_NAME}" -n "octomap"
  echo -e "${GREEN}  [octomap] octomap_robot_mapping.launch.py h:=${H}${NC}"
  tmux send-keys -t "${SESSION_NAME}:octomap" "${TMUX_SOURCE}" Enter
  tmux send-keys -t "${SESSION_NAME}:octomap" \
    "ros2 launch humanoid_sim octomap_robot_mapping.launch.py h:=${H}" Enter
  sleep 2
fi

# --- [record] bag（含起始静置段：离线重放的 FAST-LIO IMU init 靠它）---
tmux new-window -t "${SESSION_NAME}" -n "record"
echo -e "${GREEN}  [record] ros2 bag record → ${BAG_DIR}/${MAP}${NC}"
tmux send-keys -t "${SESSION_NAME}:record" "${TMUX_SOURCE}" Enter
tmux send-keys -t "${SESSION_NAME}:record" \
  "echo '=== 建图录包（SOP §4 阶段 1）===
  开录后仍静置 5~10s 再推行；慢匀速 0.3~0.5 m/s、转大弯；
  死角/重要区域多扫两遍；闭环收尾：回起点、朝向对齐、静置 3s 再 Ctrl+C 停录'" Enter
tmux send-keys -t "${SESSION_NAME}:record" \
  "cd ${BAG_DIR} && ros2 bag record /livox/lidar /livox/imu -o ${MAP}" Enter

# --- 完成 ---
echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN} 真机建图已启动（MAP=${MAP}）${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo " tmux 窗口（实时序号 · 名字）:"
tmux list-windows -t "${SESSION_NAME}" -F '   #{window_index}  #{window_name}'
echo "     tilt    = 倾角读数（保持运行：结束时记「拆架前」读数）"
echo "     livox   = Livox MID-360 驱动"
echo "     fastlio = FastLIO2 现场监控（Ctrl+C 退出时存 ${FIELD_PCD}）"
if [ -z "${NO_OCTOMAP:-}" ]; then
  echo "     octomap = 倒装 z 窗 2D 投影监控"
fi
echo "     record  = bag 录制 → ${BAG_DIR}/${MAP}/"
echo ""
echo " 进窗查看/操控: tmux attach -t ${SESSION_NAME}（脚本结束后退回原终端，五窗在后台继续跑）"
echo " 脱离回终端: Ctrl+B d —— 进程不受影响，随时可再 attach；收尾按下面结束顺序逐窗 Ctrl+C"
echo " 切窗（attach 后，默认前缀 Ctrl+B）:"
echo "   Ctrl+B w    窗口列表按名字选（推荐——序号有洞/某窗崩了也不怕）"
echo "   Ctrl+B n/p  下一个/上一个窗"
echo "   Ctrl+B 0-9  按序号直达（序号见上表）"
echo "   未 attach 时查看: tmux list-windows -t ${SESSION_NAME}"
echo " 清扫: 四窗都 Ctrl+C 停完后 tmux kill-session -t ${SESSION_NAME}"
echo ""
echo -e "${YELLOW} 结束顺序（bag/pcd 的收尾都挂在 Ctrl+C 上，别用 kill-session 停）:${NC}"
echo "   1. 回起点静置 3s → [record] Ctrl+C 停录（等收尾打印，才写 metadata.yaml）"
echo "   2. [fastlio] Ctrl+C —— 等终端打出 PCD 保存完成再关窗（现场 pcd 是监控副产品）"
echo "   3. [octomap] Ctrl+C（若有）"
echo "   4. [tilt] 记「拆架前」读数（与调平读数对账，SOP §6.3 三时点）再 Ctrl+C"
echo "   5. 四窗都停完 → tmux kill-session -t ${SESSION_NAME} 清扫"
echo ""
echo " ⚠ kill-session / Ctrl+B x 关窗发的是 SIGHUP，不触发收尾："
echo "   bag 缺 metadata.yaml（ros2 bag info/play 报错）、fastlio 不存 PCD"
echo "   未 attach 时远程停窗（等价进窗 Ctrl+C）:"
echo "     tmux send-keys -t ${SESSION_NAME}:record C-c"
echo "   收尾验证: ls ${BAG_DIR}/${MAP}/metadata.yaml && ros2 bag info ${BAG_DIR}/${MAP}"
echo ""
echo -e "${YELLOW} 现场产物传回 WSL（SOP §4 阶段 2，WSL 侧先 mkdir -p ~/humanoid_ws/rosbag/${MAP}）:${NC}"
echo "   cd ${BAG_DIR}/${MAP}"
echo "   zstd -k ${MAP}_0.db3"
echo "   scp ${MAP}_0.db3.zst metadata.yaml hmj@<WSL_IP>:/home/hmj/humanoid_ws/rosbag/${MAP}/"
echo "   （${MAP}_field.pcd 为监控副产品，可一并传回复核；权威产物以 WSL 离线重放为准）"
echo ""
echo " SOP: docs/sim2real/真机建图SOP-机器人吊架native-2026-10-26.md"
