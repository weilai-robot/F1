#!/usr/bin/env bash
# No hardware, ROS2, EtherCAT privileges or full project build required.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SDK_DIR="$REPO_ROOT/motion_control/module/dcu_driver_module/xyber_controller/xyber_api"
TEST_DIR="$(mktemp -d "${TMPDIR:-/tmp}/f1-mit-test.XXXXXX")"
trap 'rm -rf "$TEST_DIR"' EXIT

"${CXX:-c++}" -std=c++20 -I "$SDK_DIR/include" \
  "$SCRIPT_DIR/tests/test_mit_encoding.cpp" "$SDK_DIR/src/power_flow.cpp" \
  -o "$TEST_DIR/test_mit_encoding"
"$TEST_DIR/test_mit_encoding"
