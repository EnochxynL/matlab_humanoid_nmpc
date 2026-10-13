#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cmake -S "$root" -B "$root/build/standalone" \
  -DG1_BUILD_CORE=ON -DG1_BUILD_MATLAB=OFF \
  -DCMAKE_DISABLE_FIND_PACKAGE_Matlab=ON \
  -DCMAKE_BUILD_TYPE=Release '-DCMAKE_CXX_FLAGS_RELEASE=-O1 -DNDEBUG' \
  -DCMAKE_PREFIX_PATH="${G1_NUMERICAL_PREFIX:-/opt/ros/jazzy}" \
  -DPYTHON_EXECUTABLE="${G1_BUILD_PYTHON:-/usr/bin/python3}" "$@"
cmake --build "$root/build/standalone" --target g1_controller_core --parallel 1
