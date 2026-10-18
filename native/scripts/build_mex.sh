#!/usr/bin/env bash
set -euo pipefail
native_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
root="$(cd "$native_root/.." && pwd)"
build="$native_root/build/linux"
profile="${G1_CONAN_HOST_PROFILE:-default}"
for dependency in blasfeo hpipm ocs2; do
  conan export "$native_root/conan/recipes/$dependency"
done
conan export "$native_root/conan/recipes/coal" --version=3.0.3
conan export "$native_root/conan/recipes/pinocchio" --version=4.1.0
conan install "$native_root/conanfile.txt" -pr:h "$profile" -pr:b "${G1_CONAN_BUILD_PROFILE:-$profile}" \
  --lockfile="$native_root/conan.lock" --build=missing -of "$build/deps" \
  --deployer="$native_root/conan/deploy_runtime.py" --deployer-folder="$build/lib"
cmake -S "$native_root" -B "$build" \
  -DCMAKE_TOOLCHAIN_FILE="$build/deps/conan_toolchain.cmake" \
  -DG1_BUILD_MATLAB=ON -DCMAKE_BUILD_TYPE=Release '-DCMAKE_CXX_FLAGS_RELEASE=-O1 -DNDEBUG' \
  -DMatlab_ROOT_DIR="${G1_MATLAB_ROOT:-/usr/local/MATLAB/R2025b}" "$@"
cmake --build "$build" --parallel "${G1_BUILD_JOBS:-2}"
cmake --install "$build" --prefix "$root/install/linux"
find "$build/lib" -maxdepth 1 -type f -name '*.so*' -exec cp -p {} "$root/install/linux/lib/" \;
