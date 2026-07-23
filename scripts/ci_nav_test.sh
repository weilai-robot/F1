#!/bin/bash
# ============================================================
# ci_nav_test.sh — CI 无人值守导航仿真测试
#
# 完整流程: Xvfb → 启动全链路 → 健康检查 → 切 walk_mode
#           → nav_test_runner.py → 收集结果 → 清理
#
# 用法:
#   ./scripts/ci_nav_test.sh                  # 全量 batch 测试
#   ./scripts/ci_nav_test.sh all              # 同上
#   ./scripts/ci_nav_test.sh A_straight_5m    # 单个场景
#   ./scripts/ci_nav_test.sh custom 5.0 0.0   # 自定义坐标
#
# 前提:
#   - 当前 shell 已执行 f1ros
#   - motion_control 已构建 (build/aimrt_main)
#   - navigation 已构建 (navigation/install/)
#
# 产出:
#   $F1_REPORT_DIR — 本次 run 的 JSON + Markdown 测试报告
#   $F1_CI_LOG_DIR — 本次 run 的各组件日志
# ============================================================
set -euo pipefail

# ── 颜色 ──────────────────────────────────────────────────
G='\033[0;32m'; Y='\033[1;33m'; R='\033[0;31m'; B='\033[0;34m'; N='\033[0m'
cecho() { echo -e "$1"; }

# ── 路径 ──────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build"
NAV_DIR="${ROOT_DIR}/navigation"
CI_LOG_DIR="${F1_CI_LOG_DIR:-${ROOT_DIR}/ci_logs/local}"
REPORT_DIR="${F1_REPORT_DIR:-${ROOT_DIR}/reports/local}"
MODEL_PATH="${BUILD_DIR}/cfg/sim_module/model/mjcf/xyber_x1_nav.xml"

mkdir -p "$CI_LOG_DIR" "$REPORT_DIR"

# ── 基础设施失败证据 ──────────────────────────────────────
record_infrastructure_failure() {
    local reason="$1"
    python3 - "$REPORT_DIR/infrastructure_failure.json" "$reason" <<'PY'
import json
import pathlib
import sys

path = pathlib.Path(sys.argv[1])
path.write_text(
    json.dumps(
        {
            "classification": "infrastructure_failure",
            "stage": "runtime_preflight",
            "reason": sys.argv[2],
            "test_outcome": "infrastructure_failure",
        },
        ensure_ascii=False,
        indent=2,
    )
    + "\n",
    encoding="utf-8",
)
PY
}

# ── 参数解析 ──────────────────────────────────────────────
SCENARIO="${1:-all}"

# ── PID 追踪 (用于 cleanup) ───────────────────────────────
PIDS=()
XVFB_PID=""

# ── cleanup 函数 ──────────────────────────────────────────
cleanup() {
    cecho "\n${Y}[ci] 清理进程...${N}"
    for pid in "${PIDS[@]:-}"; do
        kill "$pid" 2>/dev/null || true
    done
    [ -n "$XVFB_PID" ] && kill "$XVFB_PID" 2>/dev/null || true
    # Safety net: 杀掉可能残留的相关进程
    pkill -f "aimrt_main" 2>/dev/null || true
    pkill -f "mujoco_lidar_bridge" 2>/dev/null || true
    pkill -f "leg_odom_node" 2>/dev/null || true
    sleep 2
    cecho "${G}[ci] 清理完成${N}"
}
trap cleanup EXIT

# ── 健康检查函数 ──────────────────────────────────────────
wait_for_topic() {
    local topic="$1" timeout="$2" label="$3"
    cecho "${B}[ci] 等待 ${label} (${timeout}s)...${N}"
    local elapsed=0
    while ! ros2 topic list 2>/dev/null | grep -q "$topic"; do
        sleep 1
        elapsed=$((elapsed + 1))
        if [ "$elapsed" -ge "$timeout" ]; then
            cecho "${R}[ci] ✗ 超时: ${label} (${topic})${N}"
            return 1
        fi
    done
    cecho "${G}[ci] ✓ ${label} 就绪${N}"
    return 0
}

wait_for_action() {
    local action="$1" timeout="$2" label="$3"
    cecho "${B}[ci] 等待 ${label} (${timeout}s)...${N}"
    local elapsed=0
    while ! ros2 action list 2>/dev/null | grep -q "$action"; do
        sleep 1
        elapsed=$((elapsed + 1))
        if [ "$elapsed" -ge "$timeout" ]; then
            cecho "${R}[ci] ✗ 超时: ${label} (${action})${N}"
            return 1
        fi
    done
    cecho "${G}[ci] ✓ ${label} 就绪${N}"
    return 0
}

check_pid_alive() {
    local pid="$1" label="$2"
    if ! kill -0 "$pid" 2>/dev/null; then
        cecho "${R}[ci] ✗ ${label} 进程已退出! 日志:${N}"
        cecho "${R}--- $(basename "$3") 最后 20 行 ---${N}"
        tail -20 "$3" 2>/dev/null || echo "(无日志)"
        return 1
    fi
    return 0
}

# ============================================================
#  1. 环境准备
# ============================================================
cecho "\n${G}═══════════════════════════════════════════${N}"
cecho "${G} CI 导航仿真测试 — 场景: ${SCENARIO}${N}"
cecho "${G}═══════════════════════════════════════════${N}"

cecho "\n${B}[ci] 1/6 环境准备${N}"
if ! command -v ros2 >/dev/null 2>&1; then
    cecho "${R}[ERROR] ROS2 环境未激活。请先在当前终端执行 f1ros。${N}"
    exit 1
fi
if ! python3 -c "import rclpy" >/dev/null 2>&1; then
    cecho "${R}[ERROR] 当前 Python 环境缺少 rclpy。请先执行 f1ros。${N}"
    exit 1
fi
cecho "${G}  ✓ f1ros 环境: python=$(command -v python3), ROS_DISTRO=${ROS_DISTRO:-unknown}${N}"

# Source ROS2 + navigation workspace
source "$SCRIPT_DIR/ros2_source.sh"
if [ -f "${NAV_DIR}/install/setup.bash" ]; then
    _prev_opts="$(set +o)"; set +u
    source "${NAV_DIR}/install/setup.bash"
    eval "$_prev_opts"
    cecho "${G}  ✓ navigation workspace${N}"
else
    cecho "${R}[ERROR] navigation 未构建: ${NAV_DIR}/install/setup.bash${N}"
    exit 1
fi

# 验证产物
if [ ! -f "${BUILD_DIR}/aimrt_main" ]; then
    cecho "${R}[ERROR] motion_control 未构建: ${BUILD_DIR}/aimrt_main${N}"
    cecho "${Y}  请先运行: ./scripts/build.sh${N}"
    exit 1
fi
if [ ! -f "$MODEL_PATH" ]; then
    cecho "${R}[ERROR] 导航场景模型缺失: $MODEL_PATH${N}"
    exit 1
fi
cecho "${G}  ✓ 构建产物验证通过${N}"

# 编译成功不代表插件可在 Runner 上加载。检查所有动态依赖，避免把
# glibc/共享库不兼容错误误判成导航算法失败。
PLUGIN_PATH="${BUILD_DIR}/libpkg1.so"
if [ ! -f "$PLUGIN_PATH" ]; then
    reason="motion_control plugin missing: ${PLUGIN_PATH}"
    record_infrastructure_failure "$reason"
    cecho "${R}[ERROR] ${reason}${N}"
    exit 2
fi
LDD_OUTPUT="$(ldd "$PLUGIN_PATH" 2>&1 || true)"
if echo "$LDD_OUTPUT" | grep -Eq "version .+ not found|not found"; then
    reason="motion_control runtime dependency check failed: $(echo "$LDD_OUTPUT" | grep -E "version .+ not found|not found" | head -n 1)"
    record_infrastructure_failure "$reason"
    cecho "${R}[ERROR] ${reason}${N}"
    exit 2
fi
cecho "${G}  ✓ motion_control 动态依赖可加载${N}"

# ============================================================
#  2. 启动 Xvfb (无头渲染)
# ============================================================
cecho "\n${B}[ci] 2/6 启动 Xvfb 虚拟显示${N}"

# 检查 Xvfb 是否安装
if ! command -v Xvfb &>/dev/null; then
    cecho "${R}[ERROR] Xvfb 未安装${N}"
    cecho "${Y}  请安装: sudo apt install -y xvfb${N}"
    exit 1
fi

# 清理可能残留的锁文件 (上次 CI 非正常退出)
rm -f /tmp/.X99-lock /tmp/.X11-unix/X99 2>/dev/null || true
# 清理残留进程
pkill -f "Xvfb :99" 2>/dev/null || true
sleep 0.5

export DISPLAY=:99
Xvfb :99 -screen 0 1280x720x24 -ac +extension GLX +render -noreset \
    > "$CI_LOG_DIR/xvfb.log" 2>&1 &
XVFB_PID=$!
sleep 2
if ! kill -0 "$XVFB_PID" 2>/dev/null; then
    cecho "${R}[ERROR] Xvfb 启动失败${N}"
    cecho "${Y}--- Xvfb 日志 ---${N}"
    cat "$CI_LOG_DIR/xvfb.log" 2>/dev/null || echo "(无日志)"
    cecho "${Y}--- 诊断 ---${N}"
    cecho "  Xvfb 路径: $(which Xvfb)"
    cecho "  DISPLAY: ${DISPLAY}"
    ls -la /tmp/.X*-lock /tmp/.X11-unix/ 2>/dev/null || echo "  无 X 锁文件"
    exit 1
fi
cecho "${G}  ✓ Xvfb (PID=${XVFB_PID}, DISPLAY=${DISPLAY})${N}"

# 清理 ROS2 daemon 状态 (避免上次残留 topic 列表)
ros2 daemon stop 2>/dev/null || true
sleep 1
ros2 daemon start 2>/dev/null || true

# ============================================================
#  3. 启动仿真全链路
# ============================================================
cecho "\n${B}[ci] 3/6 启动仿真全链路${N}"

# [1] aimrt_main (运动控制 + MuJoCo 物理)
cecho "  启动 aimrt_main..."
(
    cd "$BUILD_DIR"
    set +eu  # 关闭 -u/-e，避免 source ROS2 相关文件时未绑定变量导致退出
    source install/share/ros2_plugin_proto/local_setup.bash 2>/dev/null || true
    exec ./aimrt_main --cfg_file_path=./cfg/x1_cfg_sim_nav.yaml
) > "$CI_LOG_DIR/aimrt.log" 2>&1 &
PIDS+=($!)

# 健康检查: 等待 ground truth topic
if ! wait_for_topic "/mujoco/ground_truth" 30 "aimrt_main + MuJoCo"; then
    cecho "${R}[ERROR] aimrt_main 启动失败${N}"
    cecho "${Y}--- aimrt.log 最后 30 行 ---${N}"
    tail -30 "$CI_LOG_DIR/aimrt.log" 2>/dev/null || echo "(无日志)"
    exit 1
fi
check_pid_alive "${PIDS[-1]}" "aimrt_main" "$CI_LOG_DIR/aimrt.log" || exit 1

# [2] MuJoCo LiDAR Bridge
cecho "  启动 mujoco_lidar_bridge..."
export MUJOCO_LIDAR_SRC="${NAV_DIR}/sim/MuJoCo-LiDAR/src"
python3 "${NAV_DIR}/planning/humanoid_sim/scripts/mujoco_lidar_bridge.py" \
    --ros-args -p model_path:="${MODEL_PATH}" \
               -p output_type:=pointcloud2 \
               -p downsample:=1 \
               -p lidar_hz:=10 \
    > "$CI_LOG_DIR/lidar_bridge.log" 2>&1 &
PIDS+=($!)
sleep 3
check_pid_alive "${PIDS[-1]}" "lidar_bridge" "$CI_LOG_DIR/lidar_bridge.log" || exit 1

# [3] FastLIO2
cecho "  启动 fast_lio..."
ros2 launch fast_lio mapping_sim_module.launch.py \
    > "$CI_LOG_DIR/fastlio.log" 2>&1 &
PIDS+=($!)

if ! wait_for_topic "/Odometry" 20 "FastLIO2"; then
    cecho "${R}[ERROR] FastLIO2 启动失败${N}"
    cecho "${Y}--- fastlio.log 最后 30 行 ---${N}"
    tail -30 "$CI_LOG_DIR/fastlio.log" 2>/dev/null || echo "(无日志)"
    exit 1
fi

# [4] open3d_loc (ICP 全局定位)
cecho "  启动 open3d_loc..."
ros2 launch open3d_loc open3d_loc_x1.launch.py use_sim_time:=true \
    > "$CI_LOG_DIR/open3d_loc.log" 2>&1 &
PIDS+=($!)
sleep 3

# [5] Nav2 导航栈
cecho "  启动 nav2..."
ros2 launch humanoid_sim navigation.launch.py \
    > "$CI_LOG_DIR/nav2.log" 2>&1 &
PIDS+=($!)

if ! wait_for_action "navigate_to_pose" 30 "Nav2"; then
    cecho "${R}[ERROR] Nav2 启动失败${N}"
    cecho "${Y}--- nav2.log 最后 30 行 ---${N}"
    tail -30 "$CI_LOG_DIR/nav2.log" 2>/dev/null || echo "(无日志)"
    exit 1
fi

# [6] 腿里程计
cecho "  启动 leg_odom..."
ros2 run humanoid_sim leg_odom_node.py \
    --ros-args -p model_path:="${MODEL_PATH}" \
    > "$CI_LOG_DIR/leg_odom.log" 2>&1 &
PIDS+=($!)
sleep 2

cecho "${G}  ✓ 全链路启动完成 (${#PIDS[@]} 个进程)${N}"

# ============================================================
#  4. 切换到 walk_mode
# ============================================================
cecho "\n${B}[ci] 4/6 切换 walk_mode${N}"

# 持续发布 walk_mode ~3s (与 send_nav_goal.sh 同策略)
ros2 topic pub -r 5 /walk_mode std_msgs/msg/Float32 "{data: 0.0}" \
    > /dev/null 2>&1 &
WALK_PUB_PID=$!
sleep 3
kill "$WALK_PUB_PID" 2>/dev/null || true

cecho "${Y}  等待行走稳定 (5s)...${N}"
sleep 5
cecho "${G}  ✓ 已进入 walk_mode${N}"

# ============================================================
#  5. 执行导航测试
# ============================================================
cecho "\n${B}[ci] 5/6 执行导航测试${N}"
cecho "  场景: ${SCENARIO}"

TEST_EXIT_CODE=0
if [ "$SCENARIO" = "all" ]; then
    python3 "${SCRIPT_DIR}/nav_test_runner.py" --batch --report-dir "$REPORT_DIR" \
        || TEST_EXIT_CODE=$?
elif [ "$SCENARIO" = "custom" ]; then
    GOAL_X="${2:-5.0}"
    GOAL_Y="${3:-0.0}"
    python3 "${SCRIPT_DIR}/nav_test_runner.py" \
        --goal-x "$GOAL_X" --goal-y "$GOAL_Y" --report-dir "$REPORT_DIR" \
        || TEST_EXIT_CODE=$?
else
    python3 "${SCRIPT_DIR}/nav_test_runner.py" \
        --scenario "$SCENARIO" --report-dir "$REPORT_DIR" \
        || TEST_EXIT_CODE=$?
fi

# ============================================================
#  6. 结果摘要
# ============================================================
cecho "\n${B}[ci] 6/6 测试结果${N}"

if [ -f "$REPORT_DIR/latest.json" ]; then
    cecho "${G}  报告目录: ${REPORT_DIR}/${N}"
    ls -la "$REPORT_DIR"/ 2>/dev/null | tail -10
elif ls "$REPORT_DIR"/*.json 1>/dev/null 2>&1; then
    cecho "${G}  报告目录: ${REPORT_DIR}/${N}"
    ls -la "$REPORT_DIR"/ 2>/dev/null | tail -10
else
    cecho "${R}  ✗ 未找到测试报告${N}"
fi

# 打印 batch 汇总表 (如果存在)
BATCH_SUMMARY="$(ls -t "$REPORT_DIR"/batch_summary_*.json 2>/dev/null | head -1)"
if [ -n "$BATCH_SUMMARY" ]; then
    cecho "\n${G}═══════════════════════════════════════════${N}"
    python3 -c "
import json, sys
with open('$BATCH_SUMMARY') as f:
    results = json.load(f)
print(f\"{'场景':<25} {'成功':<6} {'摔倒':<6} {'碰撞':<6} {'误差(m)':<10} {'漂移(m)':<10}\")
print('─' * 75)
for r in results:
    m = r.get('metrics', {})
    name = r.get('scenario', '?')
    succ = '✅' if m.get('success') else '❌'
    fall = '是' if m.get('fall') else '否'
    col = str(m.get('collisions', '?'))
    err = str(m.get('position_error_m', '?'))
    drift = str(m.get('drift_mean_m', '?'))
    print(f'{name:<25} {succ:<6} {fall:<6} {col:<6} {err:<10} {drift:<10}')
pass_count = sum(1 for r in results if r.get('metrics', {}).get('success'))
print(f'\n通过率: {pass_count}/{len(results)}')
" 2>/dev/null || cecho "${Y}  (汇总解析失败，请查看 JSON 文件)${N}"
    cecho "${G}═══════════════════════════════════════════${N}"
fi

cecho "\n${Y} 日志目录: ${CI_LOG_DIR}/${N}"

# ── 组件日志诊断 ──────────────────────────────────────────
cecho "\n${B}[ci] 组件日志诊断${N}"
DIAG_JSON="${REPORT_DIR}/ci_diagnostic.json"

python3 -c "
import json, os, re, glob

ci_logs = '${CI_LOG_DIR}'
components = {
    'aimrt':         'aimrt.log',
    'lidar_bridge':  'lidar_bridge.log',
    'fastlio':       'fastlio.log',
    'open3d_loc':    'open3d_loc.log',
    'nav2':          'nav2.log',
    'leg_odom':      'leg_odom.log',
}

diag = {'component_logs': {}}

for comp, fname in components.items():
    fpath = os.path.join(ci_logs, fname)
    if not os.path.exists(fpath):
        continue

    errors = []
    warnings = []
    try:
        with open(fpath, errors='replace') as f:
            for line in f:
                # ROS2 / Python 常见日志级别关键词
                line_stripped = line.strip()
                if not line_stripped or len(line_stripped) > 500:
                    continue
                rl = line_stripped.lower()
                if re.search(r'\berror\b|\bfail|\bexception|\btraceback|\babort|\bfatal', rl):
                    errors.append(line_stripped[-300:])
                elif re.search(r'\bwarn', rl):
                    warnings.append(line_stripped[-300:])
    except Exception:
        pass

    diag['component_logs'][comp] = {
        'errors':   errors[:10],     # 最多保留 10 条
        'warnings': warnings[:5],    # 最多保留 5 条
        'error_count': len(errors),
        'warn_count': len(warnings),
    }

    icon = '✅' if not errors else '❌'
    print(f'  {icon} {comp:<15} errors={len(errors):>3}  warnings={len(warnings):>3}')

with open('${DIAG_JSON}', 'w') as f:
    json.dump(diag, f, indent=2, ensure_ascii=False)
print(f'  诊断报告: ${DIAG_JSON}')
" 2>/dev/null || cecho "${Y}  (日志诊断收集失败)${N}"

# 从 JSON 结果判断导航测试是否通过 (nav_test_runner 自身 exit=0 不代表导航成功)
NAV_PASS=true
if [ "$TEST_EXIT_CODE" -ne 0 ]; then
    NAV_PASS=false
fi
if [ "$SCENARIO" = "all" ]; then
    BATCH_SUMMARY="$(ls -t "$REPORT_DIR"/batch_summary_*.json 2>/dev/null | head -1)"
    if [ -n "$BATCH_SUMMARY" ]; then
        FAIL_COUNT=$(python3 -c "
import json
with open('$BATCH_SUMMARY') as f: r=json.load(f)
print(sum(1 for x in r if not x.get('metrics',{}).get('success')))
" 2>/dev/null || echo "1")
        [ "$FAIL_COUNT" -gt 0 ] && NAV_PASS=false
    else
        NAV_PASS=false
    fi
else
    LATEST_JSON="$(ls -t "$REPORT_DIR"/"${SCENARIO}"_*.json 2>/dev/null | head -1)"
    if [ -n "$LATEST_JSON" ]; then
        SUCCESS=$(python3 -c "
import json
with open('$LATEST_JSON') as f: r=json.load(f)
print(str(r.get('metrics',{}).get('success',False)).lower())
" 2>/dev/null || echo "false")
        [ "$SUCCESS" != "true" ] && NAV_PASS=false
    else
        NAV_PASS=false
    fi
fi

if [ "$NAV_PASS" = true ]; then
    cecho "${G}[ci] ✓ 导航测试通过${N}"
    exit 0
else
    cecho "${R}[ci] ✗ 导航测试失败 (详见上方指标)${N}"
    cecho "${Y}  报告: ${REPORT_DIR}/${N}"
    exit 1
fi
