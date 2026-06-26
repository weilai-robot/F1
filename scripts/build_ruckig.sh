#!/bin/bash
# ============================================================
# build_ruckig.sh — 从源码编译 ruckig (解决 GLIBC 2.32 兼容性)
#
# 预编译的 libruckig.so 需要 GLIBC 2.32 (Ubuntu 22.04)。
# 旧系统 (如 Ubuntu 20.04, GLIBC 2.31) 需要本地编译。
#
# 用法:
#   ./build_ruckig.sh
# ============================================================
set -ex

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
RUCKIG_DIR="${ROOT_DIR}/motion_control/module/control_module/third_party"

echo "=== 编译 ruckig (源码) ==="

cd /tmp
rm -rf ruckig
git clone --depth 1 https://github.com/pantor/ruckig.git
cd ruckig

mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_INSTALL_PREFIX=/usr/local \
         -DRUCKIG_BUILD_TESTS=OFF \
         -DRUCKIG_BUILD_EXAMPLES=OFF

make -j$(nproc)
sudo make install

# 覆盖 third_party 中的预编译 .so
echo "=== 替换 third_party 中的 libruckig.so ==="
cp /usr/local/lib/libruckig.so "${RUCKIG_DIR}/lib/libruckig.so"

# 同时复制头文件 (确保版本一致)
if [ -d /usr/local/include/ruckig ]; then
    rm -rf "${RUCKIG_DIR}/include/ruckig"
    cp -r /usr/local/include/ruckig "${RUCKIG_DIR}/include/ruckig"
fi

echo ""
echo "✅ ruckig 编译安装完成"
echo "   库: ${RUCKIG_DIR}/lib/libruckig.so"
echo "   头文件: ${RUCKIG_DIR}/include/ruckig/"
echo ""
echo "现在需要重新编译 motion_control:"
echo "  cd ${ROOT_DIR} && rm -rf build/ && scripts/build.sh"
