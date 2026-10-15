#!/bin/bash
# ============================================================
# run_amp_real.sh — AMP 全身策略实机运行脚本
#
# 流程: (可选构建) → tmux 启动小脑 → 反馈检查 →
#       zero → stand(人工确认) → amp(立即接管) → 键盘模式/速度控制
# 默认 rl_x1.yaml 为固定 [1,0,0] 指令，Guide 手柄流程直接使用 build/run.sh。
# 有 constant_cmd 时速度键无效，停止必须切出 AMP；删此配置后才可调速。
#
# 键盘遥控 (AMP 模式下):
#   模式:  m=/amp_mode  s=/stand_mode  k=/keep_mode(AMP内立即退出)
#          z=/zero_mode
#   速度(/cmd_vel_limiter, 50Hz 持续发布, 训练范围内限幅):
#     ↑/↓ 前后 +0.1   ←/→ 或 A/D 左右移   Q/E 偏航
#     空格 速度归零    Ctrl+C 先零速再 stand 安全退出
#
# 用法:
#   ./run_amp_real.sh --build      # 先构建(scripts/build.sh)再启动
#   ./run_amp_real.sh              # 直接启动小脑+遥控
#   ./run_amp_real.sh --attach     # 小脑已在跑, 仅进入模式切换+遥控
#
# ⚠ 安全前提 (doc/amp_deploy.zh_CN.md):
#   - 首次联调必须在吊架/悬空保护下, 人手不离急停
#   - 默认进入 AMP 立即持续 1m/s；外部速度模式才可先零速观察再逐级加速
#   - 禁止首试强推/急转/后退边界指令
#   - 硬件急停独立可用, 勿依赖软件退出
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

BUILD_FIRST=0
ATTACH_ONLY=0
for arg in "$@"; do
  case "${arg}" in
    --build)  BUILD_FIRST=1 ;;
    --attach) ATTACH_ONLY=1 ;;
    *) echo "[WARN] 未知参数: ${arg}" ;;
  esac
done

# 可调项
PUB_RATE="${MODE_PUB_RATE:-5}"       # 模式 topic 发布频率 Hz
PUB_SEC="${MODE_PUB_S:-2.0}"         # 模式 topic 持续时长 s (防 DDS 发现丢包)
STAND_HOLD_S="${STAND_HOLD_S:-3.0}"  # stand 后等待站稳
SESSION_NAME="f1_amp"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'

# ── 1. 可选构建 ────────────────────────────────────────────
if [ "${BUILD_FIRST}" = 1 ]; then
  echo -e "${GREEN}[1/4] 构建 motion_control ...${NC}"
  bash "${SCRIPT_DIR}/build.sh"
fi

# ── 2. 定位 AimRT 运行目录 (构建产物优先, 回退仓库 install) ──
AIMRT_CANDIDATES=("${AIMRT_DIR:-}" "${ROOT_DIR}/build" "${ROOT_DIR}/motion_control/install/linux/bin")
AIMRT_BIN=""
for d in "${AIMRT_CANDIDATES[@]}"; do
  [ -z "${d}" ] && continue
  if [ -x "${d}/aimrt_main" ] && [ -f "${d}/cfg/x1_cfg.yaml" ]; then
    AIMRT_BIN="${d}"; break
  fi
done
if [ -z "${AIMRT_BIN}" ]; then
  echo -e "${RED}[ERROR] 未找到 aimrt_main + cfg/x1_cfg.yaml${NC}"
  echo "  试过: ${ROOT_DIR}/build , ${ROOT_DIR}/motion_control/install/linux/bin"
  echo "  可: export AIMRT_DIR=... 或先 --build"
  exit 1
fi
echo -e "${GREEN}[2/4] AimRT 目录: ${AIMRT_BIN}${NC}"

# 检查 AMP 模型与配置就位
ONNX_REL="cfg/control_module/policy/x1_amp_v70b_m16000_50hz.onnx"
if [ ! -f "${AIMRT_BIN}/${ONNX_REL}" ]; then
  echo -e "${RED}[ERROR] 缺少 AMP 模型: ${AIMRT_BIN}/${ONNX_REL}${NC}"
  exit 1
fi
grep -q "x1_amp_v70b_m16000_50hz.onnx" "${AIMRT_BIN}/cfg/control_module/rl_x1.yaml" \
  || { echo -e "${RED}[ERROR] rl_x1.yaml 未指向 v70b m16000b 模型${NC}"; exit 1; }
AMP_CONSTANT_CMD="$(sed -n 's/^[[:space:]]*constant_cmd:[[:space:]]*\(\[[^]]*\]\).*/\1/p' \
  "${AIMRT_BIN}/cfg/control_module/rl_x1.yaml")"

# ── 3. ROS2 环境 (与 switch_x1_mode.sh 一致: 须 source AimRT 侧) ──
source "${SCRIPT_DIR}/ros2_source.sh" 2>/dev/null \
  || source /opt/ros/humble/setup.bash
pushd "${AIMRT_BIN}" >/dev/null
[ -f ./ros2_setup.sh ] && source ./ros2_setup.sh
popd >/dev/null

# ── 4. 启动小脑 (tmux) ─────────────────────────────────────
if [ "${ATTACH_ONLY}" != 1 ]; then
  if ! command -v tmux >/dev/null 2>&1; then
    echo -e "${RED}[ERROR] 需要 tmux${NC}"; exit 1
  fi
  tmux has-session -t "${SESSION_NAME}" 2>/dev/null && {
    echo -e "${YELLOW}tmux 会话 ${SESSION_NAME} 已存在, 复用${NC}"
  } || {
    echo -e "${GREEN}[3/4] 启动小脑 (tmux: ${SESSION_NAME})${NC}"
    tmux new-session -d -s "${SESSION_NAME}" -n aimrt \
      "cd '${AIMRT_BIN}' && bash run.sh; echo '=== 小脑退出, 回车关闭 ==='; read"
    # setcap 需 sudo; run.sh 内部已自带, 等待启动
    sleep 5
  }
else
  echo -e "${GREEN}[3/4] --attach: 跳过小脑启动${NC}"
fi
tmux attach-session -t "${SESSION_NAME}" 2>/dev/null || true

# ── 5. 反馈健康检查 (AmpReady 前置条件) ────────────────────
echo -e "${GREEN}[4/4] 检查 /joint_states 与 /imu/data 反馈 ...${NC}"
check_topic_alive() {
  local topic="$1" type="$2"
  if ! ros2 topic info "${topic}" 2>/dev/null | grep -q 'Subscription count: [1-9]'; then
    echo -e "${RED}  [FAIL] ${topic} 无订阅者 —— 小脑未就绪或 ros2 后端未打开${NC}"
    return 1
  fi
  local hz
  hz="$(timeout 4 ros2 topic hz "${topic}" --window 20 2>/dev/null | head -1 || true)"
  echo -e "  ${topic}: ${hz:-无数据}"
  [ -n "${hz}" ] || return 1
}
FEEDBACK_OK=1
check_topic_alive /joint_states sensor_msgs/msg/JointState || FEEDBACK_OK=0
check_topic_alive /imu/data sensor_msgs/msg/Imu || FEEDBACK_OK=0
[ "${FEEDBACK_OK}" = 1 ] || {
  echo -e "${YELLOW}反馈检查未通过 —— /amp_mode 入口会被 AmpReady() 拒绝${NC}"
  echo -e "${YELLOW}排查: tmux attach -t ${SESSION_NAME} 看小脑日志${NC}"
  exit 1
}

# ── 模式发布 (持续发布防丢包, 对齐 switch_x1_mode.sh) ────────
pub_mode() {
  local topic="$1" label="$2"
  echo -e "${GREEN}→ ${label} (/${topic}, ${PUB_RATE}Hz × ${PUB_SEC}s)${NC}"
  ros2 topic pub -r "${PUB_RATE}" "${topic}" std_msgs/msg/Float32 "{data: 1.0}" >/dev/null 2>&1 &
  local pid=$!
  sleep "${PUB_SEC}"
  kill "${pid}" 2>/dev/null || true; wait "${pid}" 2>/dev/null || true
}

# ── 交互序列: zero → stand → (确认) → amp ──────────────────
echo ""
echo -e "${YELLOW}━━━ 安全确认 ━━━${NC}"
echo "  1. 吊架/悬空保护就位, 急停可触及 (首次联调强制)"
echo "  2. 小脑日志无 ERROR (tmux attach -t ${SESSION_NAME})"
read -r -p "确认后回车继续 (Ctrl+C 取消)..."

pub_mode zero_mode  "zero_mode (阻尼/复位)"
sleep "${STAND_HOLD_S}"
pub_mode stand_mode "stand_mode (站立)"

echo -e "${YELLOW}确认机器人已站稳、姿态正常后继续${NC}"
if [ -n "${AMP_CONSTANT_CMD}" ]; then
  read -r -p "回车进入 AMP (将立即持续执行 ${AMP_CONSTANT_CMD}，零速键不能停止，须切出 AMP)... "
else
  read -r -p "回车进入 AMP (策略将立即接管，先零速观察)... "
fi

pub_mode amp_mode "amp_mode (AMP 全身策略)"

echo ""
if [ -n "${AMP_CONSTANT_CMD}" ]; then
  echo -e "${GREEN}已进入 AMP，固定指令 ${AMP_CONSTANT_CMD} 持续生效。速度键和空格归零无效。${NC}"
else
  echo -e "${GREEN}已进入 AMP，策略立即接管。保持零速 ≥1s 观察，随后用键盘逐级加速。${NC}"
fi
echo -e "${YELLOW}退出: s(stand) / k(keep, AMP 内立即生效) / 物理急停${NC}"
sleep 1

# ── 键盘遥控 (模式 + 限幅速度, Ctrl+C 安全退出) ─────────────
python3 - "$PUB_RATE" "$AMP_CONSTANT_CMD" <<'PYEOF'
import sys, tty, termios, select, atexit
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

PUB_RATE = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
CONSTANT_CMD = sys.argv[2] if len(sys.argv) > 2 else ''

# 训练指令范围 (rl_x1.yaml amp_conf, C++ 侧还会再限幅)
VX_LIM, VX_MIN, VY_LIM, WZ_LIM = 2.5, -0.5, 0.5, 1.5
STEP = 0.1

MODES = {'z': '/zero_mode', 's': '/stand_mode', 'm': '/amp_mode',
         'k': '/keep_mode', 'i': '/idle_mode'}

HELP = """
━━━ AMP 遥控 (50Hz 持续发布 /cmd_vel_limiter) ━━━
  模式: m=amp  s=stand  k=keep(AMP内立即退出)  z=zero  i=idle
  速度: ↑/↓ 前后   ←/→ 左右移   q/e 偏航 (步进 %.1f)
        空格=归零   h=帮助
  限幅: vx[%.1f,%.1f] vy[%.1f,%.1f] wz[%.1f,%.1f]
  Ctrl+C: 先零速再 stand 安全退出
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
""" % (STEP, VX_MIN, VX_LIM, -VY_LIM, VY_LIM, -WZ_LIM, WZ_LIM)
if CONSTANT_CMD:
    HELP = f"""
━━━ AMP 固定指令 {CONSTANT_CMD} ━━━
  模式: m=amp  s=stand  k=keep(AMP内立即退出)  z=zero  i=idle
  速度键/空格归零无效；停止必须切出 AMP
  Ctrl+C: 触发 /stand_mode 退出策略
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

rclpy.init()
node = Node('amp_teleop')
vel_pub = node.create_publisher(Twist, '/cmd_vel_limiter', 10)
mode_pubs = {t: node.create_publisher(Float32, t, 1) for t in set(MODES.values())}
cmd = Twist()
dead = False

# 主循环每 0.02s 一轮 (select 超时驱动): 处理按键 + 50Hz 发布速度
def publish_vel():
    cmd.linear.x = max(VX_MIN, min(VX_LIM, cmd.linear.x))
    cmd.linear.y = max(-VY_LIM, min(VY_LIM, cmd.linear.y))
    cmd.angular.z = max(-WZ_LIM, min(WZ_LIM, cmd.angular.z))
    vel_pub.publish(cmd)

def trigger(topic):
    m = Float32(); m.data = 1.0
    for _ in range(int(PUB_RATE * 2)):  # 持续2s防丢包
        mode_pubs[topic].publish(m)
    print(f"  → 已触发 {topic}")

def stop_all():
    global cmd
    cmd = Twist()
    for _ in range(5):
        vel_pub.publish(cmd)

fd = sys.stdin.fileno()
old = termios.tcgetattr(fd)

def restore():
    global dead
    if dead: return
    dead = True
    stop_all()
    trigger('/stand_mode')  # 固定指令模式忽略零速度，依靠切到 stand 退出策略
    termios.tcsetattr(fd, termios.TCSADRAIN, old)
    print("\n[退出] 已发零速 + /stand_mode")

atexit.register(restore)
tty.setraw(fd)
print(HELP)

try:
    while rclpy.ok():
        r, _, _ = select.select([sys.stdin], [], [], 0.02)  # 50Hz 节拍
        if r:
            ch = sys.stdin.read(1)
            if ch == '\x1b':  # 方向键 ESC 序列
                seq = sys.stdin.read(2) if select.select([sys.stdin], [], [], 0.05)[0] else ''
                ch = {'[A': 'up', '[B': 'down', '[C': 'right', '[D': 'left'}.get(seq, '')
            low = ch.lower()
            if CONSTANT_CMD and (ch in ('up', 'down', 'left', 'right', ' ') or low in ('q', 'e')):
                print("  [固定指令] 速度键无效，请按 s/k 切出 AMP")
                continue
            if   ch == 'up':    cmd.linear.x += STEP
            elif ch == 'down':  cmd.linear.x -= STEP
            elif ch == 'left':  cmd.linear.y += STEP
            elif ch == 'right': cmd.linear.y -= STEP
            elif low == 'q':    cmd.angular.z += STEP
            elif low == 'e':    cmd.angular.z -= STEP
            elif ch == ' ':     cmd = Twist(); print("  [零速]")
            elif low in MODES:  trigger(MODES[low])
            elif low == 'h':    print(HELP)
            elif ch == '\x03':  break  # Ctrl+C
            if not CONSTANT_CMD:
                print(f"  vx={cmd.linear.x:+.2f} vy={cmd.linear.y:+.2f} wz={cmd.angular.z:+.2f}")
        publish_vel()
        rclpy.spin_once(node, timeout_sec=0)
finally:
    restore()
    node.destroy_node()
    rclpy.shutdown()
PYEOF

echo ""
echo -e "${GREEN}遥控已退出。如需再进 AMP: ${SCRIPT_DIR}/run_amp_real.sh --attach${NC}"
