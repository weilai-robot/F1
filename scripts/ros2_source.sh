#!/bin/bash
# ============================================================
#  ros2_source.sh — ROS2 环境自动探测与 source
#
#  按优先级搜索 ROS2 humble setup.bash:
#    1. 环境变量 ROS2_SETUP_PATH (用户手动指定)
#    2. AMENT_PREFIX_PATH 已设置 (conda env 已激活)
#    3. /opt/ros/humble/setup.bash (apt 标准安装)
#    4. ~/Anaconda/envs/*/ros_humble/setup.bash (conda 环境)
#    5. ~/anaconda3/envs/*/ros_humble/setup.bash
#    6. ~/miniconda3/envs/*/ros_humble/setup.bash
#    7. ros2 命令反推
#
#  用法: source scripts/ros2_source.sh
# ============================================================

# ── 颜色 ──────────────────────────────────────────────────
_GREEN='\033[0;32m'
_YELLOW='\033[1;33m'
_RED='\033[0;31m'
_NC='\033[0m'

ROS2_FOUND=""

# 1. 用户通过环境变量指定
if [ -n "${ROS2_SETUP_PATH:-}" ] && [ -f "${ROS2_SETUP_PATH}" ]; then
    ROS2_FOUND="${ROS2_SETUP_PATH}"
fi

# 2. AMENT_PREFIX_PATH 已设置（conda env 已激活 / 已手动 source）
if [ -z "$ROS2_FOUND" ] && [ -n "${AMENT_PREFIX_PATH:-}" ]; then
    # 检查是否已包含 humble 相关路径
    _ros_candidate="$(echo "$AMENT_PREFIX_PATH" | tr ':' '\n' | head -1)/../setup.bash"
    if [ -f "$_ros_candidate" ]; then
        ROS2_FOUND="$_ros_candidate"
    fi
    # conda env 内通常是 $CONDA_PREFIX/setup.bash
    if [ -z "$ROS2_FOUND" ] && [ -n "${CONDA_PREFIX:-}" ] && [ -f "${CONDA_PREFIX}/setup.bash" ]; then
        ROS2_FOUND="${CONDA_PREFIX}/setup.bash"
    fi
fi

# 3. 标准 apt 安装路径
if [ -z "$ROS2_FOUND" ] && [ -f /opt/ros/humble/setup.bash ]; then
    ROS2_FOUND="/opt/ros/humble/setup.bash"
fi

# 4-6. conda 环境通配搜索
if [ -z "$ROS2_FOUND" ]; then
    for _base in "$HOME/Anaconda" "$HOME/anaconda3" "$HOME/miniconda3" "/opt/conda"; do
        if [ -d "$_base/envs" ]; then
            _hit="$(find "$_base/envs" -maxdepth 2 -name setup.bash -path '*/ros_humble/setup.bash' 2>/dev/null | head -1)"
            if [ -n "$_hit" ]; then
                ROS2_FOUND="$_hit"
                break
            fi
        fi
    done
fi

# 7. ros2 命令反推
if [ -z "$ROS2_FOUND" ] && command -v ros2 &>/dev/null; then
    _ros_root="$(dirname "$(dirname "$(command -v ros2)")")"
    if [ -f "$_ros_root/setup.bash" ]; then
        ROS2_FOUND="$_ros_root/setup.bash"
    fi
fi

# ── source 或报错 ─────────────────────────────────────────
if [ -n "$ROS2_FOUND" ]; then
    # ROS2 setup.bash 引用了 AMENT_TRACE_SETUP_FILES 等未初始化变量，
    # 在 set -u 下会报错，临时关闭 nounset
    _prev_opts="$(set +o)"
    set +u
    source "$ROS2_FOUND"
    eval "$_prev_opts"
    export ROS2_FOUND
    echo -e "${_GREEN}[ros2]${_NC} ${ROS2_FOUND}"
else
    echo -e "${_RED}[ERROR] 未找到 ROS2 Humble${_NC}"
    echo -e "${_YELLOW}已搜索:${_NC}"
    echo "  - ROS2_SETUP_PATH 环境变量: ${ROS2_SETUP_PATH:-<未设置>}"
    echo "  - AMENT_PREFIX_PATH: ${AMENT_PREFIX_PATH:-<未设置>}"
    echo "  - /opt/ros/humble/setup.bash"
    echo "  - ~/Anaconda/envs/*/ros_humble/setup.bash"
    echo "  - ~/anaconda3/envs/*/ros_humble/setup.bash"
    echo "  - ~/miniconda3/envs/*/ros_humble/setup.bash"
    echo ""
    echo -e "${_YELLOW}解决方法:${_NC}"
    echo "  export ROS2_SETUP_PATH=/path/to/your/ros2/setup.bash"
    echo "  然后重新运行构建脚本"
    exit 1
fi
