#!/bin/bash
# ============================================================
# F1 Navigation Sim — Docker Entrypoint
#
# 功能:
#   1. source ROS2 环境
#   2. source navigation install
#   3. 检测显示环境 → 有 X11 就用, 没有就启动 Xvfb
#   4. 执行用户命令
# ============================================================
set -e

# --- 1. ROS2 ---
source /opt/ros/humble/setup.bash

# --- 2. Navigation workspace ---
if [ -f /workspace/navigation/install/setup.bash ]; then
    source /workspace/navigation/install/setup.bash
fi

# --- 3. 显示环境自动检测 ---
if [ -z "$DISPLAY" ] || ! ls /tmp/.X11-unix/ >/dev/null 2>&1; then
    echo "[entrypoint] 未检测到 X11 显示, 启动 Xvfb 虚拟显示 :99 ..."
    Xvfb :99 -screen 0 1920x1080x24 -ac +extension GLX +render -noreset &
    export DISPLAY=:99
    sleep 1
    echo "[entrypoint] Xvfb 已启动, DISPLAY=$DISPLAY"
else
    echo "[entrypoint] 使用 X11 显示: DISPLAY=$DISPLAY"
    # 允许容器内访问 X server (需要 host 端 xhost +local:docker)
fi

# --- 4. 执行用户命令 ---
exec "$@"
