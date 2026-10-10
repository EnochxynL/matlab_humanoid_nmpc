#!/usr/bin/env bash
# Keep the runtime change local to this MATLAB process. Pinocchio from Jazzy
# needs GLIBCXX_3.4.32, newer than MATLAB R2025b's bundled libstdc++.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
matlab_root="${G1_MATLAB_ROOT:-/usr/local/MATLAB/R2025b}"
stdlib="$(g++ -print-file-name=libstdc++.so.6)"
test -f "$stdlib"
# MATLAB also ships an older OctoMap with the same SONAME. Pinocchio's Coal
# needs the system version. Load the matching pair before MATLAB's toolboxes.
export LD_PRELOAD="$stdlib:/usr/lib/x86_64-linux-gnu/liboctomath.so.1.9:/usr/lib/x86_64-linux-gnu/liboctomap.so.1.9:/usr/lib/x86_64-linux-gnu/libgomp.so.1${LD_PRELOAD:+:$LD_PRELOAD}"
exec "$matlab_root/bin/matlab" -desktop -sd "$root" -r "if ~matlab.engine.isEngineShared, matlab.engine.shareEngine('G1Inprocess'); end; shareMATLABSession; open_inprocess;"
