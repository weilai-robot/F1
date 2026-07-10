#!/bin/bash
# ============================================================
#  motion_control 构建脚本
#  F1/CMakeLists.txt 只包含 motion_control 子目录，
#  因此本脚本等价于 motion_control 专用构建。
#
#  用法:
#    ./scripts/build.sh           Release 构建（增量）
#    ./scripts/build.sh clean     清理后全新构建
#    ./scripts/build.sh Debug     指定构建类型
# ============================================================
set -euo pipefail

# ── 颜色 ──────────────────────────────────────────────────
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# ── 参数解析 ──────────────────────────────────────────────
BUILD_TYPE="Release"
CLEAN=false
EXTRA_FLAGS=""

for arg in "$@"; do
    case $arg in
        clean)           CLEAN=true ;;
        Debug|Release|RelWithDebInfo) BUILD_TYPE="$arg" ;;
        *)               EXTRA_FLAGS="$EXTRA_FLAGS $arg" ;;
    esac
done

# ── 定位项目根目录 ────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT_DIR"

# ── source ROS2 环境 ──────────────────────────────────────
source "${SCRIPT_DIR}/ros2_source.sh"

if [ -f "${SCRIPT_DIR}/url_gitee.bashrc" ]; then
    source "${SCRIPT_DIR}/url_gitee.bashrc"
fi

# ── clean ────────────────────────────────────────────────
if [ "$CLEAN" = true ]; then
    echo -e "${YELLOW}[build] 清理 build/${NC}"
    rm -rf build
fi

# ── configure ─────────────────────────────────────────────
echo -e "${GREEN}[build] CMake configure (BUILD_TYPE=${BUILD_TYPE})${NC}"
cmake -B build \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DCMAKE_INSTALL_PREFIX=./build/install \
    -DXYBER_X1_INFER_BUILD_TESTS=OFF \
    -DXYBER_X1_INFER_SIMULATION=ON \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    ${EXTRA_FLAGS}

# ── build ─────────────────────────────────────────────────
JOBS="$(nproc 2>/dev/null || echo 4)"
BUILD_START=$(date +%s)

echo -e "${GREEN}[build] 编译 (${JOBS} 线程)${NC}"
rm -rf ./build/install
cmake --build build --config "${BUILD_TYPE}" --target install --parallel "${JOBS}"

BUILD_END=$(date +%s)
BUILD_DURATION=$((BUILD_END - BUILD_START))

# ── 验证产物 ──────────────────────────────────────────────
echo ""
echo -e "${GREEN}[build] 验证产物 ...${NC}"
MISSING=0

check() {
    if [ -e "build/$1" ]; then
        echo -e "  ${GREEN}✓${NC}  $1"
    else
        echo -e "  ${RED}✗${NC}  $1  (缺失!)"
        MISSING=$((MISSING + 1))
    fi
}

# 核心产物（编译 + custom_target + install to CMAKE_BINARY_DIR）
check aimrt_main
check libpkg1.so
check run.sh
check run_with_recording.sh
check cfg/x1_cfg.yaml
check cfg/control_module/rl_x1.yaml
check cfg/control_module/policy/rl_walk_leg.onnx
check cfg/control_module/policy/rl_walk_leg_shoulder.onnx
check cfg/dcu_driver_module/dcu_x1.yaml

if [ "$MISSING" -gt 0 ]; then
    echo ""
    echo -e "${RED}[build] ✗ ${MISSING} 个产物缺失，请检查 CMake install 规则${NC}"
    exit 1
fi

# ── 摘要 ──────────────────────────────────────────────────
echo ""
echo -e "${GREEN}══════════════════════════════════════════${NC}"
echo -e "${GREEN} motion_control 构建成功${NC}"
echo -e "${GREEN}══════════════════════════════════════════${NC}"
echo    " aimrt_main:   $(ls -lh build/aimrt_main 2>/dev/null | awk '{print $5}')"
echo    " libpkg1.so:   $(ls -lh build/libpkg1.so 2>/dev/null | awk '{print $5}')"
echo    " cfg:          $(find build/cfg -name '*.yaml' | wc -l) yamls"
echo    " onnx:         $(find build/cfg -name '*.onnx' | wc -l) models"
echo -e " 耗时:         ${BUILD_DURATION}s"
echo ""
echo -e "${YELLOW} 真机运行:${NC}"
echo    "   cd build"
echo    "   sudo setcap cap_net_raw=ep ./aimrt_main"
echo    "   bash run.sh"
echo ""
