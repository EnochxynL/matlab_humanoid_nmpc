#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=1
export MAKEFLAGS=-j1
flags='-DBOOST_MPL_LIMIT_LIST_SIZE=30 -DBOOST_MPL_LIMIT_VECTOR_SIZE=30 -DBOOST_MPL_CFG_NO_PREPROCESSED_HEADERS -DBOOST_FUSION_INVOKE_MAX_ARITY=10 -DBOOST_FUSION_INVOKE_PROCEDURE_MAX_ARITY=10 -DBOOST_FUSION_INVOKE_FUNCTION_OBJECT_MAX_ARITY=10 -isystem /opt/ros/jazzy/include/pinocchio/deprecated'
colcon --log-base "$root/build/colcon_log" build \
 --base-paths "$root/third_party/ros2" "$root/ros2_ws/src" \
 --build-base "$root/build/ros2" --install-base "$root/install" \
 --executor sequential \
 --cmake-args -DCMAKE_BUILD_TYPE=Release '-DCMAKE_CXX_FLAGS_RELEASE=-O1 -DNDEBUG' \
 "-DCMAKE_CXX_FLAGS=$flags" -DBUILD_TESTING=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
 "-DFETCHCONTENT_SOURCE_DIR_BLASFEODOWNLOAD=$root/third_party/blasfeo" \
 "-DFETCHCONTENT_SOURCE_DIR_HPIPMDOWNLOAD=$root/third_party/hpipm"
source "$root/install/local_setup.bash"
cmake -S "$root" -B "$root/build/controller" -DCMAKE_BUILD_TYPE=Release '-DCMAKE_CXX_FLAGS_RELEASE=-O1 -DNDEBUG' "-DCMAKE_CXX_FLAGS=$flags"
cmake --build "$root/build/controller" --target g1_ocp_mex -j1
