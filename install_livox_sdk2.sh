#!/bin/bash
# ============================================================
# install_livox_sdk2.sh — 安装 Livox SDK2 (仿真 + 真机共用)
#
# 在服务器上执行:
#   chmod +x install_livox_sdk2.sh
#   ./install_livox_sdk2.sh
# ============================================================
set -ex

echo "=== 安装 Livox SDK2 ==="

# 依赖
sudo apt-get update
sudo apt-get install -y cmake build-essential

# 克隆 SDK
cd /tmp
rm -rf Livox-SDK2
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2

# 构建
mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install

# 验证
echo ""
echo "=== 验证安装 ==="
ls -la /usr/local/lib/liblivox_lidar_sdk_shared.so
ls -la /usr/local/include/livox_lidar_api.h

echo ""
echo "✅ Livox SDK2 安装完成"
echo "   库: /usr/local/lib/liblivox_lidar_sdk_shared.so"
echo "   头文件: /usr/local/include/livox_lidar_api.h"
