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

# ── 补丁: 修复 GCC13 + C++20 编译错误 (缺少 <cstdint>) ──

# 补丁 1: sdk_core/comm/define.h — std::uint8_t / std::uint16_t
sed -i '1i #include <cstdint>' sdk_core/comm/define.h

# 补丁 2: sdk_core/logger_handler/file_manager.h — uint64_t
sed -i '1i #include <cstdint>' sdk_core/logger_handler/file_manager.h

# 补丁 3: 其他可能缺 cstdint 的头文件
for f in \
    sdk_core/device_manager.h \
    sdk_core/params_check.h \
    sdk_core/parse_cfg_file.h \
    sdk_core/data_handler/data_handler.h \
    sdk_core/command_handler/command_impl.h \
    sdk_core/command_handler/general_command_handler.h \
    sdk_core/base/logging.h; do
    if [ -f "$f" ] && ! grep -q '<cstdint>' "$f"; then
        sed -i '1i #include <cstdint>' "$f"
    fi
done

echo "=== 补丁完成, 开始构建 ==="

# 构建
mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_POLICY_VERSION_MINIMUM=3.5
make -j$(nproc)
sudo make install

# 验证
echo ""
echo "=== 验证安装 ==="
ls -la /usr/local/lib/liblivox_lidar_sdk_shared.so
ls -la /usr/local/include/livox_lidar_api.h

echo ""
echo "✅ Livox SDK2 安装完成"
