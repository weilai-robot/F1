#!/bin/bash
# ============================================================
# Navigation 模块构建脚本
# 基于 ROS2 colcon 构建系统
# ============================================================
set -ex

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# ── source 环境 ──────────────────────────────────────────
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

NAV_DIR="${SCRIPT_DIR}/navigation"
cd "$NAV_DIR"

# 1. 构建 ROS2 C++ / Python packages (colcon)
colcon build --symlink-install \
    --packages-select \
        livox_ros_driver2 \
        fast_lio \
        open3d_loc \
        ros2_livox_simulation \
        humanoid_sim \
    "$@"

echo "✓ Navigation ROS2 packages built (colcon)"

# 2. MuJoCo-LiDAR (pip package, 仅仿真用)
if [ -d MuJoCo-LiDAR ]; then
    echo "Installing MuJoCo-LiDAR (pip)..."
    pip install -e MuJoCo-LiDAR 2>/dev/null || echo "  (skip: pip install failed or already installed)"
fi

# 3. agibot_x1_train (仅 RL 训练用，按需手动安装)
# pip install -e agibot_x1_train

echo ""
echo "=== Navigation build complete ==="
echo "Source the workspace:"
echo "  source ${NAV_DIR}/install/setup.bash"
