#!/usr/bin/env bash
set -euo pipefail
native_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
root="$(cd "$native_root/.." && pwd)"
# This prefix locates numerical packages only. No ROS setup or colcon is used.
# On this machine Pinocchio/Coal/URDFDOM are distributed under /opt/ros/jazzy.
cmake -S "$native_root" -B "$root/build/native-linux" \
  -DG1_BUILD_CORE=ON -DG1_BUILD_MATLAB=ON -DCMAKE_DISABLE_FIND_PACKAGE_Matlab=OFF -DCMAKE_BUILD_TYPE=Release '-DCMAKE_CXX_FLAGS_RELEASE=-O1 -DNDEBUG' \
  -DCMAKE_PREFIX_PATH="${G1_NUMERICAL_PREFIX:-/opt/ros/jazzy}" \
  -DPYTHON_EXECUTABLE="${G1_BUILD_PYTHON:-/usr/bin/python3}" \
  -DMatlab_ROOT_DIR="${G1_MATLAB_ROOT:-/usr/local/MATLAB/R2025b}" "$@"
cmake --build "$root/build/native-linux" --target g1_ocp_mex --parallel 1
