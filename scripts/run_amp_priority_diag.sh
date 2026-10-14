#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
DIAG_BIN="${AIMRT_DIR:-$ROOT_DIR/build}"
MODE="${1:-help}"
if [ "$#" -gt 0 ]; then shift; fi
case "$MODE" in
  help|--help|-h)
    cat <<'HELP'
用法：
  bash scripts/run_amp_priority_diag.sh prepare --scenario static --dry-run
  bash scripts/run_amp_priority_diag.sh start --scenario static   # 先静态查左臂
  bash scripts/run_upper_direction_test.sh test --joint left_shoulder_pitch_joint
  bash scripts/run_upper_direction_test.sh diagnose
  bash scripts/run_amp_priority_diag.sh start --scenario amp      # 反馈恢复后采集 AMP
  bash scripts/run_amp_priority_diag.sh analyze --dcu-log /path/dcu_diag.csv \
    --walk-log /path/walk_diag.csv --out /tmp/amp_priority
start 默认 static；AMP 仍使用原控制参数和 Guide 指令。启动前先停止旧控制进程。
HELP
    exit 0 ;;
  analyze) exec python3 "$SCRIPT_DIR/amp_priority_diag.py" analyze "$@" ;;
  prepare|start) ;;
  *) echo "未知操作 $MODE" >&2; exit 1 ;;
esac
if [ "$MODE" = start ]; then
  if pgrep -x aimrt_main >/dev/null; then echo "请先停止旧 aimrt_main。" >&2; exit 1; fi
  if [ ! -x "$DIAG_BIN/aimrt_main" ] || [ ! -f "$DIAG_BIN/libpkg1.so" ]; then
    echo "缺少构建产物，请先 bash scripts/build.sh。" >&2; exit 1
  fi
  if ! strings "$DIAG_BIN/libpkg1.so" | grep -F 'DCU diagnostics completed:' >/dev/null; then
    echo "二进制缺少本次诊断功能，请重新构建。" >&2; exit 1
  fi
fi
if [ "$MODE" = start ]; then
  for arg in "$@"; do
    if [ "$arg" = --dry-run ]; then echo "--dry-run 请用于 prepare。" >&2; exit 1; fi
  done
fi
python3 "$SCRIPT_DIR/amp_priority_diag.py" prepare --bin-dir "$DIAG_BIN" "$@"
if [ "$MODE" = prepare ]; then exit 0; fi
DIAG_SCENARIO="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["main_config"])' "$DIAG_BIN/cfg/amp_priority_manifest.json")"
source "$SCRIPT_DIR/ros2_source.sh"
cd "$DIAG_BIN"
export AIMRT_CFG_FILE="./$DIAG_SCENARIO"
exec bash ./run.sh
