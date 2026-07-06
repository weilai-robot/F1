#!/bin/bash
# ============================================================
# build_nav.sh — Navigation 模块构建脚本 (conda colcon 版)
#
# 解决: 系统 colcon (Python 3.8) 与 conda 环境 (Python 3.12)
#       pyparsing/setuptools 冲突。
# 修复: 使用 conda 内 pip 安装的 colcon (Python 3.12) 构。
#
# 用法:
#   ./build_nav.sh              # 全量构建
#   ./build_nav.sh --no-livox   # 跳过 livox_ros_driver2 (无 SDK 时)
# ============================================================
set -ex

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
NAV_DIR="${ROOT_DIR}/navigation"

# ── source 环境 ──────────────────────────────────────────
if [ -z "${ROS_SETUP_BASH}" ]; then
    if [ -f "${CONDA_PREFIX:-}/ros_humble/setup.bash" ]; then
        ROS_SETUP_BASH="${CONDA_PREFIX}/ros_humble/setup.bash"
    elif [ -f "${CONDA_PREFIX:-}/setup.bash" ]; then
        ROS_SETUP_BASH="${CONDA_PREFIX}/setup.bash"
    elif [ -f /opt/ros/humble/setup.bash ]; then
        ROS_SETUP_BASH="/opt/ros/humble/setup.bash"
    fi
fi

if [ -z "${ROS_SETUP_BASH}" ] || [ ! -f "${ROS_SETUP_BASH}" ]; then
    echo -e "\033[0;31m[ERROR] 未找到 ROS2 setup.bash\033[0m"
    echo -e "  请设置: export ROS_SETUP_BASH=/path/to/setup.bash"
    exit 1
fi

echo -e "\033[0;32m[build_nav] ROS2: ${ROS_SETUP_BASH}\033[0m"
source "${ROS_SETUP_BASH}"

# ── 选择 colcon ──────────────────────────────────────────
# 优先使用 conda 内的 colcon, 回退到系统 colcon
CONDA_COLCON=$(which colcon 2>/dev/null)
SYSTEM_COLCON="/usr/bin/colcon"

if [ -x "${CONDA_COLCON}" ]; then
    COLCON="${CONDA_COLCON}"
    echo -e "\033[0;32m[build_nav] 使用 conda colcon: ${COLCON}\033[0m"
elif [ -x "${SYSTEM_COLCON}" ]; then
    COLCON="${SYSTEM_COLCON}"
    echo -e "\033[0;33m[build_nav] 警告: 使用系统 colcon (${COLCON})\033[0m"
    echo -e "\033[0;33m         如果遇到 'type' object is not subscriptable 错误,\033[0m"
    echo -e "\033[0;33m         请在 conda 环境中安装: pip install colcon-common-extensions\033[0m"
else
    echo -e "\033[0;31m[ERROR] colcon 未找到, 请安装:\033[0m"
    echo -e "  conda 环境: pip install colcon-common-extensions"
    echo -e "  系统:       sudo apt install python3-colcon-common-extensions"
    exit 1
fi

# ── 参数解析 ──────────────────────────────────────────────
BUILD_LIVOX=true
for arg in "$@"; do
    case $arg in
        --no-livox) BUILD_LIVOX=false ;;
    esac
done

cd "$NAV_DIR"

# ── 1. 构建 ROS2 C++ / Python packages ──────────────────
PACKAGES=""
if [ "$BUILD_LIVOX" = true ]; then
    PACKAGES="livox_ros_driver2 fast_lio humanoid_sim"
else
    PACKAGES="fast_lio humanoid_sim"
fi

echo -e "\033[0;32m[build_nav] 构建包: ${PACKAGES}\033[0m"
${COLCON} build --symlink-install \
    --packages-select ${PACKAGES} \
    --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble \
    "$@"

source "${NAV_DIR}/install/setup.bash"
HUMANOID_SIM_PREFIX="$(ros2 pkg prefix humanoid_sim)"
ODOM_BRIDGE_EXE="${HUMANOID_SIM_PREFIX}/lib/humanoid_sim/odom_bridge.py"
if [ ! -x "${ODOM_BRIDGE_EXE}" ]; then
    echo -e "\033[0;31m[ERROR] odom_bridge.py 未正确安装: ${ODOM_BRIDGE_EXE}\033[0m"
    exit 1
fi

echo -e "\033[0;32m✓ Navigation ROS2 packages 构建成功\033[0m"

# ── 2. MuJoCo-LiDAR (pip package, 仅仿真用) ──────────────
if [ -d sim/MuJoCo-LiDAR ]; then
    echo -e "\033[0;32m[build_nav] 安装 MuJoCo-LiDAR (pip)...\033[0m"
    pip install -e sim/MuJoCo-LiDAR 2>/dev/null || \
        echo -e "\033[0;33m  ⚠ MuJoCo-LiDAR pip install 跳过\033[0m"
fi

echo ""
echo -e "\033[0;32m=== Navigation build complete ===\033[0m"
echo "Source the workspace:"
echo "  source ${NAV_DIR}/install/setup.bash"
