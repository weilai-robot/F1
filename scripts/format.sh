#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT_DIR"

# clang-format, version v15 is required
find ./motion_control -regex '.*\.cc\|.*\.h\|.*\.proto' -and -not -regex '.*\.pb\.cc\|.*\.pb\.h' | xargs clang-format -i --style=Google
echo "clang-format done"

# cmake-format, apt install cmake-format
{ find . -maxdepth 1 -name "CMakeLists.txt"; find ./motion_control -name "CMakeLists.txt"; } | xargs cmake-format -c ./.cmake-format.py -i
{ find ./cmake -name "*.cmake"; find ./motion_control -name "*.cmake"; } | xargs cmake-format -c ./.cmake-format.py -i
echo "cmake-format done"

# autopep8, apt install python3-autopep8
{ find . -maxdepth 1 -name "*.py" -print; find ./motion_control -name "*.py" -print; } | xargs autopep8 -i --global-config ./.pycodestyle
echo "python format done"
