#!/bin/bash
# ============================================================
# run_loco_perf.sh — 运动控制 (MuJoCo 仿真) 基础性能测试一键执行
#
# 覆盖四项测试 (通过 /cmd_vel_limiter 发送 vx/vy/wz):
#   1. straight  直线: 最大速度(不摔倒) / 达成率 / 是否走直线 / 偏移率
#   2. turn      原地转弯: 最大角速度(不摔倒) / 达成率 / 是否偏移原地 / 偏移率
#   3. arc       弧线: 最大执行速度(不摔倒) / 达成率 / 半径误差 / 轨迹残差
#   4. stop      停止: 0.4m/s 停止时间 / 最大减速度
#
# 行为:
#   1. 校验 build 产物, 自动生成 build 下的 perf 仿真配置副本
#      (sim_x1_perf.yaml + x1_cfg_sim_perf.yaml; 重新 build 后由源码配置接管)
#   2. tmux 启动 aimrt_main (平地场景 + ground truth), 等待就绪
#   3. 逐项运行 scripts/loco_perf_test.py (默认每项测试前重启仿真, 保证初始状态干净)
#   4. 汇总产出报告
#
# 用法:
#   ./scripts/run_loco_perf.sh                      # 全部 4 项
#   ./scripts/run_loco_perf.sh --tests straight,stop
#   ./scripts/run_loco_perf.sh --no-sim             # 附着到已运行的仿真 (不启停)
#   ./scripts/run_loco_perf.sh --keep-sim           # 跑完保留仿真窗口
#   ./scripts/run_loco_perf.sh --levels "0.2,0.4,0.6,0.8" --hold 5
#
# 产出: reports/loco_<test>_<时间戳>.{json,md,gt.csv,cmd.csv}
# ============================================================
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build"
REPORT_DIR="${ROOT_DIR}/reports"
SESSION="f1_loco_perf"

TESTS="straight,turn,arc,stop"
MANAGED=1            # 1=本脚本管理仿真; 0=--no-sim 附着
KEEP_SIM=0
RESTART_EACH=1
LEVELS=""
HOLD=""
EXTRA=""
WORST=0

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'; NC='\033[0m'

usage() {
    cat <<'EOF'
用法:
  ./scripts/run_loco_perf.sh [选项]

选项:
  --tests LIST      要跑的测试项 (逗号分隔, 默认 straight,turn,arc,stop)
  --no-sim          附着到已运行的仿真 (不自动启停; 需已发布 /mujoco/ground_truth)
  --keep-sim        测试完成后保留仿真 tmux 会话 (便于现场查看)
  --no-restart      多项测试之间不重启仿真 (注意: 上一项若摔倒, 后续数据无效)
  --report-dir DIR  报告目录 (默认 F1/reports)
  --levels STR      档位序列透传 (straight/turn: "0.2,0.4,..."; arc: "0.2:0.2,...")
  --hold N          每档保持时间 sim 秒 (默认 6)
  --extra "ARGS"    追加透传给 loco_perf_test.py 的其它参数
  -h, --help        显示帮助

产出: reports/loco_<test>_<时间戳>.{json,md,gt.csv,cmd.csv}
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --tests)      TESTS="$2"; shift 2 ;;
        --no-sim)     MANAGED=0; shift ;;
        --keep-sim)   KEEP_SIM=1; shift ;;
        --no-restart) RESTART_EACH=0; shift ;;
        --report-dir) REPORT_DIR="$2"; shift 2 ;;
        --levels)     LEVELS="$2"; shift 2 ;;
        --hold)       HOLD="$2"; shift 2 ;;
        --extra)      EXTRA="$2"; shift 2 ;;
        -h|--help)    usage; exit 0 ;;
        *) echo -e "${RED}未知参数: $1${NC}"; usage; exit 1 ;;
    esac
done

START_TS="$(date +%s)"

echo -e "${CYAN}╔════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║  运动控制基础性能测试 (MuJoCo 仿真)           ║${NC}"
echo -e "${CYAN}╚════════════════════════════════════════════════╝${NC}"

# ── 清理钩子 ─────────────────────────────────────────────
# 正常完成 (rc=0/2): 关闭仿真会话
# 中断/abort (rc>=128 或 rc=3): 保留会话便于排查现场
cleanup() {
    local rc=$?
    if [ "${MANAGED}" != "1" ]; then
        return 0
    fi
    if [ "${KEEP_SIM}" = "1" ]; then
        if tmux has-session -t "${SESSION}" 2>/dev/null; then
            echo -e "${YELLOW}[cleanup] 仿真会话保留 (--keep-sim): tmux attach -t ${SESSION}${NC}"
        fi
        return 0
    fi
    if [ $rc -ge 128 ] || [ $rc -eq 3 ]; then
        if tmux has-session -t "${SESSION}" 2>/dev/null; then
            echo -e "${YELLOW}[cleanup] 仿真会话保留 (便于排查): tmux attach -t ${SESSION}  关闭: tmux kill-session -t ${SESSION}${NC}"
        fi
    else
        tmux kill-session -t "${SESSION}" 2>/dev/null || true
        echo -e "${GREEN}[cleanup] 仿真会话已关闭${NC}"
    fi
}
trap cleanup EXIT

# ── 前置检查 ─────────────────────────────────────────────
if ! command -v tmux >/dev/null 2>&1; then
    echo -e "${RED}[ERROR] 未找到 tmux${NC}"; exit 1
fi
if [ ! -f "${BUILD_DIR}/aimrt_main" ]; then
    echo -e "${RED}[ERROR] motion_control 未构建: ${BUILD_DIR}/aimrt_main 不存在${NC}"
    echo "  请先运行: ./scripts/build.sh"
    exit 1
fi

# ── source ROS2 (复用仓库统一探测脚本) ───────────────────
if [ -f "${SCRIPT_DIR}/ros2_source.sh" ]; then
    # shellcheck disable=SC1091
    source "${SCRIPT_DIR}/ros2_source.sh"
else
    echo -e "${RED}[ERROR] 缺少 ${SCRIPT_DIR}/ros2_source.sh${NC}"; exit 1
fi

# ── 生成/检查 build 下的 perf 配置 ───────────────────────
ensure_perf_cfgs() {
    local sim_cfg="${BUILD_DIR}/cfg/sim_module/sim_x1_perf.yaml"
    local top_cfg="${BUILD_DIR}/cfg/x1_cfg_sim_perf.yaml"

    if [ ! -f "${sim_cfg}" ]; then
        if [ ! -f "${BUILD_DIR}/cfg/sim_module/sim_x1.yaml" ]; then
            echo -e "${RED}[ERROR] ${BUILD_DIR}/cfg/sim_module/sim_x1.yaml 不存在, 请先构建${NC}"
            return 1
        fi
        cp "${BUILD_DIR}/cfg/sim_module/sim_x1.yaml" "${sim_cfg}"
        cat >> "${sim_cfg}" <<'YAML'

# [perf] 性能测试: 显式发布 ground truth / base pose
pub_base_pose_topic: /mujoco/base_pose
pub_ground_truth_topic: /mujoco/ground_truth
YAML
        echo -e "${YELLOW}[cfg] 已生成 ${sim_cfg} (sim_x1.yaml + GT 发布)${NC}"
    fi

    if [ ! -f "${top_cfg}" ]; then
        sed 's|sim_x1\.yaml|sim_x1_perf.yaml|' "${BUILD_DIR}/cfg/x1_cfg_sim.yaml" > "${top_cfg}"
        if ! grep -q "sim_x1_perf" "${top_cfg}"; then
            echo -e "${RED}[ERROR] 生成 x1_cfg_sim_perf.yaml 失败${NC}"
            return 1
        fi
        echo -e "${YELLOW}[cfg] 已生成 ${top_cfg}${NC}"
    fi
    return 0
}

# ── 仿真启动 / 就绪 / 关闭 ───────────────────────────────
launch_sim() {
    tmux kill-session -t "${SESSION}" 2>/dev/null || true
    sleep 1
    local inner="cd ${BUILD_DIR} && source ${ROS2_FOUND} && source ./install/share/ros2_plugin_proto/local_setup.bash 2>/dev/null; if [ -z \"\$DISPLAY\" ] && command -v Xvfb >/dev/null 2>&1; then Xvfb :99 -screen 0 1280x720x24 -ac +extension GLX +render -noreset >/dev/null 2>&1 & export DISPLAY=:99; sleep 1; fi; ./aimrt_main --cfg_file_path=./cfg/x1_cfg_sim_perf.yaml"
    tmux new-session -d -s "${SESSION}" -n sim
    tmux send-keys -t "${SESSION}:0" "${inner}" Enter
    echo -e "${GREEN}[sim] aimrt_main 已启动 (x1_cfg_sim_perf.yaml, 平地+GT)${NC}"
}

wait_ready() {
    local timeout="${1:-60}" i
    for i in $(seq 1 "${timeout}"); do
        if ros2 topic list 2>/dev/null | grep -q "/mujoco/ground_truth"; then
            if timeout 8 ros2 topic echo /mujoco/ground_truth --once >/dev/null 2>&1; then
                echo -e "${GREEN}[ready] 仿真就绪 (${i}s)${NC}"
                return 0
            fi
        fi
        if [ $((i % 10)) -eq 0 ]; then
            echo -e "${YELLOW}  ... 等待仿真就绪 (${i}/${timeout}s)${NC}"
        fi
        sleep 1
    done
    echo -e "${RED}[ERROR] 等待仿真就绪超时 (${timeout}s) — 检查: tmux attach -t ${SESSION}${NC}"
    return 1
}

# ── 主流程 ───────────────────────────────────────────────
if [ "${MANAGED}" = "1" ]; then
    ensure_perf_cfgs || exit 1
else
    echo -e "${YELLOW}[mode] --no-sim: 附着到已运行仿真 (需已发布 /mujoco/ground_truth)${NC}"
    if ! wait_ready 10; then
        echo -e "${RED}[ERROR] --no-sim 模式: 未检测到仿真 ground truth 输出${NC}"
        exit 3
    fi
fi

SIM_STARTED=0
cd "${ROOT_DIR}"

for t in $(echo "${TESTS}" | tr ',' ' '); do
    case "${t}" in
        straight|turn|arc|stop) ;;
        *) echo -e "${YELLOW}[skip] 未知测试项: ${t}${NC}"; continue ;;
    esac

    if [ "${MANAGED}" = "1" ]; then
        if [ "${SIM_STARTED}" = "0" ] || [ "${RESTART_EACH}" = "1" ]; then
            launch_sim
            wait_ready 60 || exit 1
            SIM_STARTED=1
            sleep 2   # 让控制/仿真链路稳定
        fi
    fi

    RUNNER_ARGS=(--test "${t}" --report-dir "${REPORT_DIR}")
    [ -n "${LEVELS}" ] && RUNNER_ARGS+=(--levels "${LEVELS}")
    [ -n "${HOLD}" ] && RUNNER_ARGS+=(--hold "${HOLD}")
    # shellcheck disable=SC2206
    [ -n "${EXTRA}" ] && RUNNER_ARGS+=(${EXTRA})

    echo ""
    echo -e "${CYAN}>>> 测试项: ${t}${NC}"
    rc=0
    python3 "${SCRIPT_DIR}/loco_perf_test.py" "${RUNNER_ARGS[@]}" || rc=$?

    if [ "${rc}" = "3" ]; then
        WORST=3
        echo -e "${RED}[done] ${t} 中止/前置条件不满足 (exit=3)${NC}"
    elif [ "${rc}" = "2" ]; then
        [ "${WORST}" != "3" ] && WORST=2
        echo -e "${YELLOW}[done] ${t} 完成但出现摔倒 (exit=2)${NC}"
        if [ "${MANAGED}" = "0" ] || [ "${RESTART_EACH}" = "0" ]; then
            echo -e "${YELLOW}  ⚠ 摔倒后机器人已倒地, 下一项测试前请手动重启仿真${NC}"
        fi
    else
        echo -e "${GREEN}[done] ${t} 完成 (exit=${rc})${NC}"
    fi
    sleep 2
done

# ── 汇总 ─────────────────────────────────────────────────
echo ""
echo -e "${CYAN}════════════════════ 本次产出 ════════════════════${NC}"
python3 - "${REPORT_DIR}" "${START_TS}" <<'PY'
import glob, json, os, sys
rdir, start_ts = sys.argv[1], float(sys.argv[2])
files = [f for f in glob.glob(os.path.join(rdir, "loco_*.json"))
         if os.path.getmtime(f) >= start_ts]
files.sort(key=os.path.getmtime)
if not files:
    print("  (无新报告)")
for f in files:
    try:
        r = json.load(open(f))
        s = r.get("summary", {})
        t = r.get("test")
        st = r.get("status")
        if t == "straight":
            head = (f"max_v={s.get('max_no_fall_vx')} m/s eta={s.get('max_no_fall_eta')} "
                    f"直线OK={s.get('straight_ok')}")
        elif t == "turn":
            head = (f"max_wz={s.get('max_no_fall_wz')} rad/s eta={s.get('max_no_fall_eta')} "
                    f"原地OK={s.get('inplace_ok')}")
        elif t == "arc":
            head = (f"max_vw={s.get('max_no_fall_vw')} radius_err={s.get('radius_err_pct')}% "
                    f"弧线OK={s.get('arc_ok')}")
        else:
            head = (f"t_stop_mean={s.get('t_stop_mean')}s decel_peak={s.get('decel_peak_max')} m/s2 "
                    f"stop_dist={s.get('stop_dist_mean')}m")
        print(f"  [{t}] {st}: {head}")
        print(f"        {os.path.basename(f)}")
    except Exception as e:
        print(f"  (读取 {os.path.basename(f)} 失败: {e})")
PY

echo ""
echo -e "${GREEN}报告目录: ${REPORT_DIR} (loco_*.md 可读报告, loco_*.json 机读数据, *.gt.csv/.cmd.csv 原始数据)${NC}"

exit ${WORST}
