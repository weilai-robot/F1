#!/bin/bash
# ============================================================
# archive_nav_run.sh — 真机 nav 测试证据两跳拉回（只读）
#
# WSL ──ssh──> 196 (robot@10.12.201.196) ──LAN ssh──> minipc (robot@192.168.112.221)
# 全程只读（find/tar czf -），远端零写操作；拉回 WSL 后解开并生成 MANIFEST+README。
#
# 默认拉取集（相对 minipc /home/robot，经 mtime 过滤）:
#   .ros/log/                                  — nav2 容器日志（wait/spin/progress fail/MPPI）
#   Robot_github/F1/build/log/                 — 小脑主日志 x1_rl_ctrl_real_nav.log
#   Robot_github/F1/build/test_logs/data_csv/  — walk_diag / ctrl_period / t_m/tm_obs
#   Robot_github/F1/navigation/localization/fast_lio2/Log/ — fastlio pos_log.txt
#   （--bag DIR 另拉 rosbag2 目录，不过滤 mtime，整目录拉）
#
# 解包布局（沿用 nav_pull_20261103 约定）:
#   Robot_github/F1/ 前缀剥掉 → build/ navigation/
#   .ros/ → home/robot/.ros/
#
# 用法:
#   ./archive_nav_run.sh <label>                 # 默认 since=180min
#   ./archive_nav_run.sh -s 60 run2              # 只拉近 60 分钟改动的文件
#   ./archive_nav_run.sh -l                      # --list 只列远端候选文件（干跑）
#   ./archive_nav_run.sh --bag Robot_github/F1/reports/nav_test_record/20261106_xxx/bag run2
# 输出:
#   F1/reports/nav_pull/<timestamp>_<label>/  {build/, home/, navigation/, ...}
#     MANIFEST.md5s + README.md（含 bag_analyze 用法行）
#
# 前提: WSL→196 免密、196→minipc 免密（okr2-196-to-minipc key）
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
OUT_ROOT="${ROOT_DIR}/reports/nav_pull"

HOP1="robot@10.12.201.196"
HOP2="robot@192.168.112.221"
SSH_OPTS="-o ConnectTimeout=10 -o BatchMode=yes"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

SINCE=180
LABEL=""
LIST_ONLY=false
BAGS=()

usage() { sed -n '2,27p' "$0"; exit "${1:-0}"; }

while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help) usage 0 ;;
    -s|--since) SINCE="${2:?missing minutes}"; shift 2 ;;
    -l|--list) LIST_ONLY=true; shift ;;
    --bag) BAGS+=("${2:?missing bag path}"); shift 2 ;;
    --*) echo -e "${RED}[ERROR] 未知选项: $1${NC}" >&2; usage 1 ;;
    *) LABEL="$1"; shift ;;
  esac
done
[ -n "${LABEL}" ] || { echo -e "${RED}[ERROR] 缺 label（如 run2）${NC}" >&2; usage 1; }
LABEL="${LABEL//[^a-zA-Z0-9_-]/_}"

# 远端文件选择命令（对 minipc 执行，全部只读）
# 分组 { ...; } 必须包住所有 find 再统一 sort：否则 `a && b | sort` 的优先级
# 会把前段 find 输出裸漏进 stdout 污染 tar 流（11-06 --bag 实测踩坑）
FIND_SETS=".ros/log Robot_github/F1/build/log Robot_github/F1/build/test_logs/data_csv Robot_github/F1/navigation/localization/fast_lio2/Log"
REMOTE_FIND="cd /home/robot && { find ${FIND_SETS} -type f -mmin -${SINCE} 2>/dev/null"

# --bag 路径相对 minipc /home/robot；record_nav_test.sh 的产物在
# Robot_github/F1/reports/nav_test_record/<stamp>_<label>/bag
# （无引号拼接：路径经两跳单引号包装，词法安全依赖 stamp_label 命名约定）
for b in "${BAGS[@]:-}"; do
  [ -n "$b" ] || continue
  REMOTE_FIND="${REMOTE_FIND}; find ${b} -type f 2>/dev/null"
done
REMOTE_FIND="${REMOTE_FIND}; } | sort"

echo -e "${YELLOW}[archive] 远端候选（minipc:/home/robot，mtime -${SINCE}min）:${NC}"
if ! ssh ${SSH_OPTS} "${HOP1}" "ssh ${SSH_OPTS} ${HOP2} '${REMOTE_FIND} | xargs -r ls -la 2>/dev/null | tail -40'"; then
  echo -e "${RED}[ERROR] 两跳 ssh 失败（WSL→196→minipc）${NC}" >&2
  exit 1
fi

N_FILES=$(ssh ${SSH_OPTS} "${HOP1}" "ssh ${SSH_OPTS} ${HOP2} '${REMOTE_FIND} | wc -l'")
if [ "${N_FILES}" -eq 0 ]; then
  echo -e "${RED}[ERROR] 远端无候选文件（栈没跑过？--since 太小？）${NC}" >&2
  exit 1
fi
echo -e "${GREEN}[archive] 候选 ${N_FILES} 个文件${NC}"

if [ "${LIST_ONLY}" = true ]; then
  echo -e "${YELLOW}[archive] --list 干跑结束，未拉取${NC}"
  exit 0
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="${OUT_ROOT}/${STAMP}_${LABEL}"
mkdir -p "${RUN_DIR}"

echo -e "${YELLOW}[archive] 拉取中 → ${RUN_DIR}/pull.tgz${NC}"
# tar czf - 经双跳管道回 WSL；--warning=no-file-changed 忽略拉取瞬间还在写的日志；
# 该情形 tar 仍退出 1（≠致命 2），故 `test $? -le 1` 放行（11-06 RViz 日志在写实测踩坑）
if ! ssh ${SSH_OPTS} "${HOP1}" "ssh ${SSH_OPTS} ${HOP2} '${REMOTE_FIND} | tar czf - -T - --warning=no-file-changed; test \$? -le 1'" > "${RUN_DIR}/pull.tgz"; then
  echo -e "${RED}[ERROR] tar 流拉取失败${NC}" >&2
  exit 1
fi

# 解包 + 路径重写（对齐 nav_pull_20261103 布局）
tar xzf "${RUN_DIR}/pull.tgz" -C "${RUN_DIR}" \
  --transform 's|^\.ros/|home/robot/.ros/|' \
  --transform 's|^Robot_github/F1/||'
rm "${RUN_DIR}/pull.tgz"

# MANIFEST（md5 对账，11-03 同法）
(cd "${RUN_DIR}" && find . -type f -print0 | sort -z | xargs -0 md5sum > MANIFEST.md5s)

cat >"${RUN_DIR}/README.md" <<EOF
# nav 证据拉回 — ${LABEL}

- **时间**: $(date --iso-8601=seconds)
- **远端**: minipc /home/robot（${HOP2}，经 ${HOP1} 两跳）
- **mtime 窗口**: 近 ${SINCE} min；bag: ${BAGS[*]:-无}
- **布局**: Robot_github/F1/ 前缀已剥（build/ navigation/）；.ros/ → home/robot/.ros/
- **校验**: MANIFEST.md5s（对账用）

## 指标提取

\`\`\`bash
python3 scripts/bag_analyze.py reports/nav_pull/${STAMP}_${LABEL}/reports/nav_test_record/*/bag --goal <X> <Y> <YAW> --dump-timeline /tmp/tl
\`\`\`

## 看什么（11-03 教训对照）

| 信号 | 健康值 | 异常含义 |
|------|--------|----------|
| exec_vx/wz_slope | ≈0.7-1.0 | ≈0 = 小脑执行塌缩；负 = 符号链问题 |
| stall_windows | 无 | 有 = 命令-响应脱节窗口（谁没动的判别器） |
| nav2 容器 log "Running spin/wait" | wait 优先 | spin = A批 BT 失效 |
| progress fail 间隔 | — | 精确 35s = movement_time_allowance 循环 |
EOF

echo ""
echo -e "${GREEN}[archive] 完成${NC}"
echo "  目录: ${RUN_DIR}"
find "${RUN_DIR}" -type f | sed "s|${RUN_DIR}/|  |" | sort
