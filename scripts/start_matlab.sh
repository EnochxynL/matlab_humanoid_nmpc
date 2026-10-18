#!/usr/bin/env bash
# Keep compiler/runtime selection local to this MATLAB process. Conan-built
# dependencies must use the matching GCC runtime rather than MATLAB's copy.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
matlab_root="${G1_MATLAB_ROOT:-/usr/local/MATLAB/R2025b}"
stdlib="$(g++ -print-file-name=libstdc++.so.6)"
test -f "$stdlib"
# Use the runtime libraries deployed with this project's Conan dependencies.
runtime="$root/install/linux/lib"
for library in liboctomath.so liboctomap.so; do test -f "$runtime/$library"; done
export LD_LIBRARY_PATH="$runtime${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export LD_PRELOAD="$stdlib:$runtime/liboctomath.so:$runtime/liboctomap.so:$(g++ -print-file-name=libgomp.so)${LD_PRELOAD:+:$LD_PRELOAD}"
exec "$matlab_root/bin/matlab" -desktop -sd "$root" -r "if ~matlab.engine.isEngineShared, matlab.engine.shareEngine('G1Inprocess'); end; shareMATLABSession; open_project;"
