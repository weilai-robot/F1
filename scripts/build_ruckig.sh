#!/bin/bash
# Build Ruckig on the current host so libruckig.so only requires the host GLIBC.
# The pinned commit was validated on the self-hosted F1 runner.

set -euo pipefail

readonly RUCKIG_REPOSITORY="https://github.com/pantor/ruckig.git"
readonly RUCKIG_COMMIT="a8db97a4e9c55e5160a3855f739fa3b270df8e4c"

if [ "${1:-}" = "--print-commit" ]; then
    printf '%s\n' "$RUCKIG_COMMIT"
    exit 0
fi
if [ "$#" -ne 0 ]; then
    echo "Usage: $0 [--print-commit]" >&2
    exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
RUCKIG_DIR="${ROOT_DIR}/motion_control/module/control_module/third_party"
RUCKIG_TARGET_LIB="${RUCKIG_DIR}/lib/libruckig.so"

TASK_TEMP_ROOT="${RUNNER_TEMP:-${TMPDIR:-/tmp}}"
if [ ! -d "$TASK_TEMP_ROOT" ]; then
    echo "ERROR: temporary root does not exist: $TASK_TEMP_ROOT" >&2
    exit 1
fi

RUCKIG_BUILD_ROOT="$(mktemp -d "${TASK_TEMP_ROOT%/}/f1-ruckig.XXXXXX")"
RUCKIG_SOURCE_DIR="${RUCKIG_BUILD_ROOT}/source"
RUCKIG_INSTALL_DIR="${RUCKIG_BUILD_ROOT}/install"

cleanup() {
    rm -rf "$RUCKIG_BUILD_ROOT"
}
trap cleanup EXIT

echo "=== Build host-compatible Ruckig ==="
echo "repository: $RUCKIG_REPOSITORY"
echo "commit:    $RUCKIG_COMMIT"
echo "host:      $(getconf GNU_LIBC_VERSION 2>/dev/null || echo unknown-glibc)"

git init --quiet "$RUCKIG_SOURCE_DIR"
git -C "$RUCKIG_SOURCE_DIR" remote add origin "$RUCKIG_REPOSITORY"
git -C "$RUCKIG_SOURCE_DIR" fetch --quiet --depth 1 origin "$RUCKIG_COMMIT"
git -C "$RUCKIG_SOURCE_DIR" checkout --quiet --detach FETCH_HEAD

ACTUAL_COMMIT="$(git -C "$RUCKIG_SOURCE_DIR" rev-parse HEAD)"
if [ "$ACTUAL_COMMIT" != "$RUCKIG_COMMIT" ]; then
    echo "ERROR: Ruckig commit mismatch: $ACTUAL_COMMIT != $RUCKIG_COMMIT" >&2
    exit 1
fi

JOBS="$(nproc 2>/dev/null || echo 4)"
cmake -S "$RUCKIG_SOURCE_DIR" -B "$RUCKIG_SOURCE_DIR/build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$RUCKIG_INSTALL_DIR" \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_PYTHON_MODULE=OFF \
    -DBUILD_CLOUD_CLIENT=OFF \
    -DBUILD_TESTS=OFF \
    -DBUILD_EXAMPLES=OFF
cmake --build "$RUCKIG_SOURCE_DIR/build" --parallel "$JOBS"
cmake --install "$RUCKIG_SOURCE_DIR/build"

RUCKIG_BUILT_LIB="$(find "$RUCKIG_INSTALL_DIR" -type f -name 'libruckig.so*' -print -quit)"
if [ -z "$RUCKIG_BUILT_LIB" ]; then
    echo "ERROR: libruckig.so was not installed under $RUCKIG_INSTALL_DIR" >&2
    exit 1
fi
if [ ! -d "$RUCKIG_INSTALL_DIR/include/ruckig" ]; then
    echo "ERROR: Ruckig headers were not installed under $RUCKIG_INSTALL_DIR" >&2
    exit 1
fi

install -m 0755 "$RUCKIG_BUILT_LIB" "$RUCKIG_TARGET_LIB"
rm -rf "${RUCKIG_DIR}/include/ruckig"
cp -R "$RUCKIG_INSTALL_DIR/include/ruckig" "${RUCKIG_DIR}/include/ruckig"

LDD_OUTPUT="$(ldd "$RUCKIG_TARGET_LIB" 2>&1 || true)"
printf '%s\n' "$LDD_OUTPUT"
if printf '%s\n' "$LDD_OUTPUT" | grep -Eq 'version .+ not found|not found'; then
    echo "ERROR: rebuilt libruckig.so still has unresolved runtime dependencies" >&2
    exit 1
fi

echo "Ruckig $RUCKIG_COMMIT is compatible with this runner."
echo "library: $RUCKIG_TARGET_LIB"
