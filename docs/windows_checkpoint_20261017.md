# Windows checkpoint — 2026-10-17

> Superseded stopping instruction: the user subsequently authorized continuing past floating-point roundoff differences, through the final walking test. The strict FAIL below remains historical evidence. The full adapter comparison subsequently found 26 differing doubles across 2441 cases, maximum absolute error 8.881784197001252e-16. All 2401 observation identities, 6 rejection cases and 2401 reference transports passed. Native Windows AD value/Jacobian/sparse-Hessian generation and fresh-process reload tests also passed; core dependency build is in progress.

## Historical outcome before continuation was authorized

The three standalone MEX adapters compile with MinGW GCC 8.1.0 and MATLAB R2025b. The first 1317 frozen observation conversions match bit for bit. Case 1318 differs in component 1 (Euler conversion output):

- actual bits: `3fb16c781c33399b`
- Linux expected bits: `3fb16c781c33399a`
- maximum absolute difference in that case: `1.3877787807814457e-17`
- differing doubles in that case: 1

Validation stops immediately at this case. This is a strict FAIL, not a toleranced PASS. No formula, optimization, solver or acceptance threshold was changed in response. Core/AD/Planner MEX build and physical simulation were not started. Observation-extraction and reference replay acceptance were not reached by this sequential gate.

## Completed

- Located the existing Conan 2.29.1 executable under the user's Local AppData installation; no duplicate Conan installation.
- Downloaded and extracted an isolated MinGW GCC 8.1.0 x86_64 POSIX/SEH toolchain into `build/windows/toolchains/mingw64`.
- Created an isolated Conan home at `build/windows/conan-home`, leaving the user's default cache/configuration untouched.
- Installed Conan Eigen 3.4.0 for the standalone adapters; saved a real recipe lock in `tools/windows/adapters.conan.lock`.
- Built a minimal MEX using MATLAB's explicit `mingw64_g++.xml`, without changing the selected MEX compiler or persistent PATH.
- Configured `adapters/matlab` independently with Conan CMakeToolchain and built all three `.mexw64` files.
- Ran `tools/windows/validate_adapters.m` in a fresh MATLAB batch process, with the GCC runtime directory on that process's PATH. The minimal probe loaded and returned 81; the conversion MEX loaded and ran until the first numerical difference.
- Reattached to the user-opened MATLAB session afterward and read the saved failure evidence. Its working directory was preserved.

## Evidence (ignored build directory)

- `build/windows/adapters-configure.log`
- `build/windows/adapters-build.log`
- `build/windows/adapters-conan.log`
- `build/windows/mex-probe-build.log`
- `build/windows/adapters-matlab.log`
- `build/windows/adapters-acceptance.json`
- `build/windows/first_conversion_difference.mat` (input, values, both bit patterns, result)
- `build/windows/adapters/` (three built MEX files)
- `build/windows/probes/` (minimal compiler/load probe)

Do not delete or overwrite this evidence when resuming. Generated binaries and downloads are not committed.

## Source acquisition and review updates

Conan Center Index checkout: `1c90a5cb2e6353488efefa7d16020c19f9fae0c5`, stored in `build/windows/conan-center-index`.

The console_bridge source review gap is resolved: its recipe is in `recipes/console_bridge/all`, checks C++14, supports shared/static, and has no explicit MinGW exclusion. Assimp's recipe is under `recipes/assimp/5.x`, not `all`; it was read locally and requires C++17 when building. These static findings are not build acceptance.

At the initial review stage, the acquired Center snapshot offered Coal 3.0.4,
not the older cached 3.0.2 from web review. The target remained 3.0.3 and had not
yet been exported or built. Continuation progress below supersedes that status.

Download SHA256 values:

| Archive | SHA256 |
|---|---|
| MinGW x86_64-8.1.0-release-posix-seh-rt_v6-rev0.7z | `853970527b5de4a55ec8ca4d3fd732c00ae1c69974cc930c82604396d43e79f8` |
| Complete Pinocchio 4.1.0 release tar.gz | `b2ac9575bd4f38e0584e0d61586a95de1aeaa17c6877e31e99f1d4b913d602d8` |
| Coal 3.0.3 release tar.gz | `d1afcc0c22477a61e93e070a01cc8ed1d256a96ec65d308844d24b9caf771d36` |

An earlier interrupted Pinocchio download was detected as truncated. Only `pinocchio-4.1.0.complete.tar.gz` is the complete archive above; the partial archive/extraction must not be used as a source input.

Sources:
- https://download.qt.io/development_releases/prebuilt/mingw_64/x86_64-8.1.0-release-posix-seh-rt_v6-rev0.7z
- https://github.com/stack-of-tasks/pinocchio/releases/download/v4.1.0/pinocchio-4.1.0.tar.gz
- https://github.com/coal-library/coal/releases/download/v3.0.3/coal-3.0.3.tar.gz

## Current resumption boundary

The user explicitly superseded the strict bitwise stopping rule and authorized
continuing through the final walking test without changing controller logic.
Preserve numerical comparison reports, but do not stop for last-bit roundoff.
Continue core/MEX build, replay and walking validation autonomously. Protocol,
dimension, finite-value, discrete mode and control-time failures still require
investigation. Do not claim a physical walking pass before running the plant.

## Continuation progress

- Assimp 5.4.3 and Coal 3.0.3 compiled and packaged with the isolated MinGW 8.1.
- Pinocchio 4.1.0 compiled and packaged. Both Coal and Pinocchio require a diagnostic-only
  `__FUNCSIG__` compiler guard correction and explicit MinGW DLL symbol exports;
  patches live in local recipes. Missing symbols were confirmed in object files,
  then existing objects were relinked and packaged with Conan `export-pkg`.
- `tools/windows/core.conan.lock` now records the resolved recipe revisions.
  Root CMake configuration succeeded; core compilation is in progress.
- The deployed 30 dependency DLLs passed the runtime import scan.
- A real OCS2 AD integration probe exposed separate CppAD tape state across PE
  DLLs: the generated polynomial returned constant 3 and zero derivatives.
  Windows internal numerical targets now link statically into the controller
  DLL; Linux retains shared targets. With this layout, the probe's generation
  and fresh-process reload both pass: value 15, Jacobian 8, Hessian 2.
  Evidence: `build/windows/ocs2-ad-static-create.log` and
  `build/windows/ocs2-ad-static-reload.log`. The earlier failed probe/cache is
  preserved as evidence and is separate from the controller's AD cache.
- The full Pinocchio release archive can be reused via `G1_PINOCCHIO_ARCHIVE`;
  its pinned SHA-256 is checked before extraction.
- MATLAB's selected C++ compiler is the isolated project MinGW 8.1.
- `validate_planner_dataflow` accepts an explicit continuation flag, reports
  baseline numerical error, and preserves stage/time/mode and same-run output checks.
- The reconnected MATLAB session reports R2025b (25.2.0.2998904).
- Windows header macros `FALSE`/`TRUE` are cleared alongside the existing macro
  cleanup to preserve OCS2's `Convergence::FALSE` enumeration. Native Win32 calls
  use BOOL value 1 for inherited handles. AD generation and fresh-process reload
  regression both pass after this change (`platform-regression.log`, 2/2).
- `run_windows_planner_replay` scopes generated Simulink files to Windows caches
  and restores the prior cache settings and working folder on exit.
- Core build completed: controller DLL, four MEX files and native replay executable.
  Incremental rebuild reports no work; runtime import scan passes for 39 binaries.
  The OCS2 AD integration probe passes generation and fresh reload again (15/8/2).
- Native replay completes 2401 ticks and 289 MPC updates. Against the accepted
  Linux CSV baseline, maximum errors are state 5.818764861692527e-6,
  input 3.4961740455230483e-4 and torque 1.6273834134850063e-4.
  Control-time identities and modes are exact; all outputs are finite.
  This is NOT a bitwise pass. Evidence:
  `build/windows/core-replay/accepted_baseline_comparison.json`.
- Actual MATLAB planner-model replay passes 2401 ticks / 289 updates and staged
  protocol guards. Model action output exactly matches the same-run C++ log.
  Linux baseline maximum error is 3.4962e-4, retained under user-authorized
  continuation. Simulation wall time: 54.6509 s. Evidence:
  `build/windows/planner_dataflow/model_replay/acceptance.json`.

## Completed Windows walking validation

- Evidence folder: `runtime/walking_20261017_152410`.
- Genuine Simscape simulation completed 4.8 s with 9601 physical output samples,
  2401 control ticks and 289 MPC updates. All existing automatic checks pass.
- Action mapping maximum error: 0. Maximum raw/canonical time difference:
  8.881784197001252e-16 s. Physical states are finite.
- Displacement: [0.379083648990471, 0.076257676097903, -0.030746348777685] m.
  Height range: [0.736608766463909, 0.8] m. Maximum absolute pitch: 0.599626870556054 deg.
- Startup (cached AD): 1.095565 s; simulation wall time: 656.0923646 s.
  The MATLAB MCP call timed out at 600 s while MATLAB kept running. Completion
  was verified through `verification.json`, the returned SimulationOutput ending
  at 4.8 s, and `physical_summary.png`; do not rerun solely for that tool timeout.
- The summary plot was inspected: stable height, bounded pitch and alternating
  support forces; no fall is apparent. The user subsequently confirmed human
  PASS. `verification.json` now records `human_visual: PASS`, and
  `runtime/current_run.json` records overall status `PASS`.
- No OCP equations, solver settings, gait/reference, drive parameters or time
  semantics were changed. Cross-platform numerical differences remain reported.
- The upstream optional SQP text logger still prints an inability to open its
  Linux `/tmp/ocs2/sqp_log/...` filename on Windows. Project solver timing and
  planner/physical evidence files were recorded successfully.
