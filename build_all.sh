#!/bin/bash
# ============================================================
# build_all.sh — 统一编译打包 (motion_control + navigation)
#
# 用法:
#   ./build_all.sh              # 全量构建
#   ./build_all.sh --mc-only    # 仅 motion_control
#   ./build_all.sh --nav-only   # 仅 navigation
#   ./build_all.sh --pack       # 构建 + 打 tar 包
#   ./build_all.sh --pack --mc-only  # 仅 motion_control 打包
#
# 产出:
#   build/                          → motion_control 产物 (aimrt_main + libpkg1.so + cfg/)
#   navigation/install/             → navigation 产物 (ROS2 packages)
#   dist/f1_deploy_<date>.tar.gz    → 可部署的打包文件 (--pack)
# ============================================================
set -ex

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

# ── 颜色 ──────────────────────────────────────────────────
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# ── 参数解析 ──────────────────────────────────────────────
BUILD_MC=true
BUILD_NAV=true
DO_PACK=false
CMAKE_FLAGS=""

for arg in "$@"; do
    case $arg in
        --mc-only)  BUILD_NAV=false ;;
        --nav-only) BUILD_MC=false ;;
        --pack)     DO_PACK=true ;;
        --debug)    CMAKE_FLAGS="-DCMAKE_BUILD_TYPE=Debug" ;;
        --no-sim)   CMAKE_FLAGS="$CMAKE_FLAGS -DXYBER_X1_INFER_SIMULATION=OFF" ;;
        *)          CMAKE_FLAGS="$CMAKE_FLAGS $arg" ;;
    esac
done

# ── 环境检查 ──────────────────────────────────────────────
echo -e "${GREEN}[build_all] 开始统一构建${NC}"

if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo -e "${RED}[ERROR] 未找到 ROS2 Humble, 请先安装${NC}"
    exit 1
fi
source /opt/ros/humble/setup.bash

if [ -f "${SCRIPT_DIR}/url_gitee.bashrc" ]; then
    source "${SCRIPT_DIR}/url_gitee.bashrc"
fi

BUILD_START=$(date +%s)

# ═══════════════════════════════════════════════════════════
#  Phase 1: motion_control (CMake + AimRT)
# ═══════════════════════════════════════════════════════════
if [ "$BUILD_MC" = true ]; then
    echo -e "\n${GREEN}━━━ Phase 1: motion_control (CMake/AimRT) ━━━${NC}"

    cmake -B build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=./build/install \
        -DXYBER_X1_INFER_BUILD_TESTS=OFF \
        -DXYBER_X1_INFER_SIMULATION=ON \
        -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
        $CMAKE_FLAGS

    if [ -d ./build/install ]; then
        rm -rf ./build/install
    fi

    cmake --build build --config Release --target install --parallel $(nproc)

    # 验证关键产物
    if [ ! -f build/aimrt_main ] || [ ! -f build/libpkg1.so ]; then
        echo -e "${RED}[ERROR] motion_control 构建产物缺失${NC}"
        ls -la build/aimrt_main build/libpkg1.so 2>/dev/null || true
        exit 1
    fi
    echo -e "${GREEN}  ✓ aimrt_main + libpkg1.so 构建成功${NC}"
    echo -e "${GREEN}  ✓ 配置文件:$(ls build/cfg/*.yaml | wc -l) 个 yaml${NC}"
fi

# ═══════════════════════════════════════════════════════════
#  Phase 2: navigation (colcon)
# ═══════════════════════════════════════════════════════════
if [ "$BUILD_NAV" = true ]; then
    echo -e "\n${GREEN}━━━ Phase 2: navigation (colcon) ━━━${NC}"

    cd "$SCRIPT_DIR/navigation"
    colcon build --symlink-install \
        --packages-select \
            livox_ros_driver2 \
            fast_lio \
            open3d_loc \
            ros2_livox_simulation \
            humanoid_sim

    echo -e "${GREEN}  ✓ ROS2 packages 构建成功${NC}"

    # MuJoCo-LiDAR
    if [ -d MuJoCo-LiDAR ]; then
        pip install -e MuJoCo-LiDAR 2>/dev/null || \
            echo -e "${YELLOW}  ⚠ MuJoCo-LiDAR pip install 跳过 (可能已安装)${NC}"
    fi

    cd "$SCRIPT_DIR"
fi

BUILD_END=$(date +%s)
BUILD_DURATION=$((BUILD_END - BUILD_START))

echo -e "\n${GREEN}━━━ 构建完成 (${BUILD_DURATION}s) ━━━${NC}"

# ═══════════════════════════════════════════════════════════
#  Phase 3: 打包 (--pack)
# ═══════════════════════════════════════════════════════════
if [ "$DO_PACK" = true ]; then
    echo -e "\n${GREEN}━━━ Phase 3: 打包 ━━━${NC}"

    DIST_DIR="${SCRIPT_DIR}/dist"
    mkdir -p "$DIST_DIR"

    DATE_TAG=$(date +%Y%m%d_%H%M%S)
    GIT_HASH=$(git rev-parse --short HEAD 2>/dev/null || echo "nogit")
    PKG_NAME="f1_deploy_${DATE_TAG}_${GIT_HASH}"
    PKG_DIR="${DIST_DIR}/${PKG_NAME}"

    mkdir -p "${PKG_DIR}"

    # --- motion_control 产物 ---
    if [ "$BUILD_MC" = true ]; then
        echo "  打包 motion_control..."
        mkdir -p "${PKG_DIR}/motion_control"
        cp build/aimrt_main "${PKG_DIR}/motion_control/"
        cp build/libpkg1.so "${PKG_DIR}/motion_control/"

        # third_party .so (运行时依赖)
        mkdir -p "${PKG_DIR}/motion_control/lib"
        find motion_control/module/*/third_party/lib -name "*.so*" -exec cp -P {} "${PKG_DIR}/motion_control/lib/" \; 2>/dev/null || true
        find motion_control/module/*/third_party/lib -name "*.so*" -exec cp -L {} "${PKG_DIR}/motion_control/lib/" \; 2>/dev/null || true

        # AimRT runtime .so
        find build/_deps/ -name "libaimrt*.so" -exec cp {} "${PKG_DIR}/motion_control/lib/" \; 2>/dev/null || true

        # 配置文件 + 脚本
        cp -r build/cfg "${PKG_DIR}/motion_control/"
        cp build/*.sh "${PKG_DIR}/motion_control/" 2>/dev/null || true
        cp build/*.py "${PKG_DIR}/motion_control/" 2>/dev/null || true

        # 模型文件 (MuJoCo XML + mesh)
        mkdir -p "${PKG_DIR}/motion_control/cfg/sim_module"
        cp -r build/cfg/sim_module/model "${PKG_DIR}/motion_control/cfg/sim_module/" 2>/dev/null || true
    fi

    # --- navigation 产物 ---
    if [ "$BUILD_NAV" = true ]; then
        echo "  打包 navigation..."
        mkdir -p "${PKG_DIR}/navigation"

        # colcon install (编译后的 ROS2 packages)
        if [ -d navigation/install ]; then
            cp -r navigation/install "${PKG_DIR}/navigation/"
        fi

        # MuJoCo-LiDAR 源码 (Python -e install 需要源码)
        if [ -d navigation/MuJoCo-LiDAR ]; then
            cp -r navigation/MuJoCo-LiDAR "${PKG_DIR}/navigation/"
        fi

        # humanoid_sim scripts (Python 脚本不走 colcon install)
        if [ -d navigation/humanoid_sim/scripts ]; then
            mkdir -p "${PKG_DIR}/navigation/humanoid_sim"
            cp -r navigation/humanoid_sim/scripts "${PKG_DIR}/navigation/humanoid_sim/"
        fi
    fi

    # --- 顶层启动脚本 ---
    echo "  打包启动脚本..."
    cp run_sim.sh "${PKG_DIR}/" 2>/dev/null || true
    cp run_sim_nav.sh "${PKG_DIR}/" 2>/dev/null || true
    cp run_nav_sim.sh "${PKG_DIR}/" 2>/dev/null || true
    cp run_nav_real.sh "${PKG_DIR}/" 2>/dev/null || true
    cp nav_test_runner.py "${PKG_DIR}/" 2>/dev/null || true
    cp build_nav.sh "${PKG_DIR}/" 2>/dev/null || true

    # --- 版本信息 ---
    cat > "${PKG_DIR}/VERSION.txt" << EOF
F1 Deploy Package
=================
Date:     $(date -u +%Y-%m-%dT%H:%M:%SZ)
Git:      ${GIT_HASH}
Built on: $(hostname)
Contents:
  motion_control/ — aimrt_main + libpkg1.so + cfg + third_party libs
  navigation/     — colcon install + MuJoCo-LiDAR + scripts
Usage:
  # 仿真
  cd motion_control && ./aimrt_main --cfg_file_path=./cfg/x1_cfg_sim.yaml
  # 导航仿真
  source navigation/install/setup.bash
  ./run_sim_nav.sh
EOF

    # --- 打 tar ---
    echo "  创建 tar.gz..."
    cd "$DIST_DIR"
    tar czf "${PKG_NAME}.tar.gz" "${PKG_NAME}"
    rm -rf "${PKG_NAME}"

    PKG_SIZE=$(du -h "${PKG_NAME}.tar.gz" | cut -f1)

    echo -e "${GREEN}  ✓ 打包完成: dist/${PKG_NAME}.tar.gz (${PKG_SIZE})${NC}"
    cd "$SCRIPT_DIR"
fi

# ═══════════════════════════════════════════════════════════
#  Summary
# ═══════════════════════════════════════════════════════════
echo ""
echo -e "${GREEN}═════════════════════════════════════════${NC}"
echo -e "${GREEN} F1 统一构建完成${NC}"
echo -e "${GREEN}═════════════════════════════════════════${NC}"

if [ "$BUILD_MC" = true ]; then
    echo -e " motion_control:"
    echo -e "   aimrt_main:  $(ls -lh build/aimrt_main 2>/dev/null | awk '{print $5}')"
    echo -e "   libpkg1.so:  $(ls -lh build/libpkg1.so 2>/dev/null | awk '{print $5}')"
    echo -e "   cfg:         $(ls build/cfg/*.yaml 2>/dev/null | wc -l) yamls"
fi

if [ "$BUILD_NAV" = true ]; then
    echo -e " navigation:"
    echo -e "   packages:    $(ls -d navigation/install/*/ 2>/dev/null | wc -l) ROS2 pkgs"
    echo -e "   source:      source navigation/install/setup.bash"
fi

if [ "$DO_PACK" = true ]; then
    echo -e " deploy pkg:"
    echo -e "   $(ls -lh dist/*.tar.gz 2>/dev/null | tail -1 | awk '{print $9, $5}')"
fi

echo -e "${GREEN} 耗时: ${BUILD_DURATION}s${NC}"
echo ""
