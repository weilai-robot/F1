#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SDK_DIR="$ROOT_DIR/motion_control/module/dcu_driver_module/xyber_controller/xyber_api"
TEST_DIR="$(mktemp -d "${TMPDIR:-/tmp}/f1-priority-test.XXXXXX")"
trap 'rm -rf "$TEST_DIR"' EXIT
"${CXX:-c++}" -std=c++20 -pthread -fsanitize=undefined -fno-sanitize-recover=all \
  -I "$SDK_DIR/include" "$SCRIPT_DIR/tests/test_dcu_diagnostics.cpp" \
  "$SDK_DIR/src/dcu.cpp" "$SDK_DIR/src/power_flow.cpp" -o "$TEST_DIR/test_dcu_diagnostics"
"$TEST_DIR/test_dcu_diagnostics"
python3 -m unittest discover -s "$SCRIPT_DIR/tests" -p 'test_amp_priority_diag.py' -v
