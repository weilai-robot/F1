#!/usr/bin/env bash
# Independent static PD direction test; see doc/上肢静态方向测试.md.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
UPPER_TEST_BIN="${AIMRT_DIR:-$ROOT_DIR/build}"
MODE="${1:-help}"
if [ "$#" -gt 0 ]; then shift; fi

if [ "$MODE" = help ] || [ "$MODE" = --help ] || [ "$MODE" = -h ]; then
    cat <<'HELP'
用法：
  bash scripts/run_upper_direction_test.sh prepare --dry-run   # 只预览
  bash scripts/run_upper_direction_test.sh start               # 终端 A，生成配置并启动
  bash scripts/run_upper_direction_test.sh start --angle 0.15  # 调整静态测试增量
  bash scripts/run_upper_direction_test.sh test                # 终端 B，逐关节交互测试
  bash scripts/run_upper_direction_test.sh test --joint left_shoulder_pitch_joint
  bash scripts/run_upper_direction_test.sh list                # 查看关节与模型方向
可用 AIMRT_DIR 指定安装目录。先停止其他 aimrt_main；机器人须有支撑。
HELP
    exit 0
fi

case "$MODE" in
    prepare|start|test|list) ;;
    *) echo "未知操作：$MODE" >&2; exit 1 ;;
esac
UPPER_TEST_BIN="$(cd "$UPPER_TEST_BIN" && pwd)"
if [ "$MODE" = list ]; then
    exec python3 "$SCRIPT_DIR/upper_direction_test.py" list \
        --manifest "$UPPER_TEST_BIN/cfg/control_module/upper_direction_test.json" "$@"
fi
if [ "$MODE" = start ]; then
    if pgrep -x aimrt_main >/dev/null; then
        echo "已有 aimrt_main，请先停止旧控制进程，再启动静态测试。" >&2
        exit 1
    fi
    if [ ! -x "$UPPER_TEST_BIN/aimrt_main" ] || [ ! -f "$UPPER_TEST_BIN/run.sh" ]; then
        echo "缺少构建产物，请先执行 bash scripts/build.sh。" >&2
        exit 1
    fi
    if ! grep -q 'AIMRT_CFG_FILE' "$UPPER_TEST_BIN/run.sh"; then
        echo "build/run.sh 尚未更新配置入口，请重新构建。" >&2
        exit 1
    fi
fi
source "$SCRIPT_DIR/ros2_source.sh"
if [ "$MODE" = prepare ] || [ "$MODE" = start ]; then
    python3 "$SCRIPT_DIR/upper_direction_test.py" prepare --bin-dir "$UPPER_TEST_BIN" "$@"
    if [ "$MODE" = prepare ]; then exit 0; fi
    cd "$UPPER_TEST_BIN"
    export AIMRT_CFG_FILE=./cfg/x1_cfg_upper_direction_test.yaml
    exec bash ./run.sh
fi

cd "$UPPER_TEST_BIN"
set +u
if [ -f ./ros2_setup.sh ]; then source ./ros2_setup.sh; fi
for prefix in ./install ../install; do
    if [ -f "$prefix/setup.bash" ]; then source "$prefix/setup.bash"; fi
    if [ -f "$prefix/share/my_ros2_proto/local_setup.bash" ]; then
        source "$prefix/share/my_ros2_proto/local_setup.bash"
    fi
done
set -u
exec python3 "$SCRIPT_DIR/upper_direction_test.py" run \
    --manifest "$UPPER_TEST_BIN/cfg/control_module/upper_direction_test.json" "$@"
