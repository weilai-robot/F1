#!/bin/bash

# exit on error and print each command
set -ex

# ── source 环境 ──────────────────────────────────────────
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
if [ -f "${SCRIPT_DIR}/url_gitee.bashrc" ]; then
    source "${SCRIPT_DIR}/url_gitee.bashrc"
fi

# cmake
cmake -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=./build/install \
    -DXYBER_X1_INFER_BUILD_TESTS=OFF \
    -DXYBER_X1_INFER_SIMULATION=ON \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    $@

if [ -d ./build/install ]; then
    rm -rf ./build/install
fi

cmake --build build --config Release --target install --parallel $(nproc)
