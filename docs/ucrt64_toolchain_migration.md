# System MSYS2 UCRT64 toolchain migration

Migration completed with system MSYS2 UCRT64 GCC 16.2.0. The default runtime at
`install/windows/` now contains the validated GCC 16 build. The previous GCC 8
deployment is preserved at `native/build/windows/previous-install-gcc8-20261018/`.
No system compiler was installed or updated by this migration.

The compiler is supplied by the user's MSYS2 installation. Project scripts
resolve `gcc`, `g++` and `objdump` from PATH and verify the x64 target and UCRT.
No compiler installation path is stored in the profiles. Conan detects the GCC
major version and includes the CRT marker in host package IDs. Build outputs
are isolated under `native/build/windows/gcc<major>-ucrt64/`; trial installation
uses `-InstallPrefix install/windows-ucrt64`.

## Completed checks

- Detected GCC 16.2.0, x64, POSIX threads, SEH, UCRT.
- CMake `matlab_add_mex` built a C++ MEX against the R2025b SDK.
  A fresh MATLAB process loaded it and returned the expected result, including
  use of the C++ standard library. Evidence: `runtime/evidence/ucrt64_migration/`.
- Boost 1.88, URDFDOM 4.0.0, HPIPM, Assimp 5.4.3, Coal 3.0.3 and
  Pinocchio 4.1.0 and OCS2 built with the new compiler.
- The Windows AD process interface launched bare `gcc` through PATH and read
  back version 16.2.0. `CreateProcess` now resolves the quoted command line
  instead of requiring an absolute application path.
- Controller DLL, Planner MEX and replay tools built and installed to the trial
  deployment. Runtime imports resolved for 34 build binaries and 32 deployed
  binaries; the build binaries all import UCRT components.
- Three Windows platform tests passed, including fresh AD generation/reload and
  diagnostic file replacement under transient and persistent file locks.
- OCS2 polynomial generation and fresh reload returned value 15, Jacobian 8,
  Hessian 2.
- A cold, compiler-specific AD cache was generated using the system GCC.
  Native replay completed 2401 ticks and 289 updates. Maximum difference from
  the Linux fixture was 0.00167215; from the previous Windows replay, 0.00201232.
- Fresh-process MATLAB model replay passed all stage guards, 2401 ticks,
  289 updates and exact policy/action agreement. Maximum difference from its
  accepted baseline was 0.0003752979835223158; from the previous Windows model
  replay, 0.0006860738351264217. These are reported differences, not bit equality.
- MATLAB code analysis reported no issues in initialization and scoped cache
  selection. New Simulink caches use `build/matlab/gcc16-ucrt64/`.

## Dependency adjustments

Abseil was declared but unused by retained targets. Its old Windows source
failed under the new toolchain, so the unused root dependency was removed.

The local URDFDOM recipe derives from ConanCenter revision
`e68d7aed7dbdd349a77e7430fe7f1f8c`. Its two existing patches allow optional
command-line applications and Conan dependency discovery. The new adaptation
adds `<cstdint>` to two headers that used fixed-width integer types without
including their declaration directly. No parsing or controller logic changes.

OCS2's recipe enables Git `core.longpaths` in its source checkout, because some
upstream example paths exceed Windows Git's legacy limit. Global Git settings
are unchanged. The Windows build script retains a `subst` mapping of
`G:` to `native/build/windows/` (configurable through `-CacheDrive`) to shorten
Conan paths; both paths refer to the same files. Generated build files use that
mapping, so it must remain available while those build trees are used.

## Windows diagnostic file replacement

Two native replays stopped after 168 and 270 samples with Windows access-denied
errors while replacing `structural_snapshot.bin`. This issue had also appeared
with the previous toolchain. The process responsible for the transient lock has
not been identified; no repository reader of this file was found.

Snapshot publication now retries Windows access/sharing/lock errors for eight
attempts, with at most 127 ms of requested backoff sleep. Persistent errors and
unrelated I/O failures still propagate. Linux keeps its original rename call.
The numerical solver and controller logic are unchanged. A deterministic test
holds the destination without delete sharing: temporary locks recover and
persistent locks retain the previous file and report failure. The subsequent
full native and MATLAB replays passed.

## Walking and active deployment

The new physical run at `runtime/walking_20261018_223121/` passed its automatic
checks and the user's human visual review:

| Check | Result |
|---|---|
| Duration / physical samples | 4.8 s / 9601 |
| Control ticks / MPC updates | 2401 / 289 |
| Action mapping maximum error | 0 |
| Forward displacement | 0.3790846397 m |
| Base height range | 0.7366087969–0.8 m |
| Maximum absolute pitch | 0.5996271575 degrees |
| Simulation wall time | 919.1753 s |
| Human visual review | PASS |

The earlier MATLAB conversion run took 614.1907 s, but this migration used fresh
compiler-specific Simulink caches and a separate MATLAB process while the
original desktop remained open. These runs are not a controlled performance
comparison; the new run does not establish a speed improvement.

After validation, the trial installation was copied into the default deployment
and its runtime metadata checked. The existing MATLAB models were stopped;
only the old Planner MEX was closed and unloaded. Its old DLLs were confirmed
unloaded before the session PATH was updated from the discovered system GCC.
The new default MEX then initialized and closed successfully in that same
session (`ACTIVE_UCRT64_RUNTIME_PASS`), with models left open. Process maps show
the new default MEX/controller and system UCRT64 GCC runtime. All four deployed
compiler runtime DLLs match the system compiler copies by SHA256.

Evidence is under `runtime/evidence/ucrt64_migration/`; the final native replay
is `core_replay_final/`, and MATLAB replay is `model_replay/`. Initial failed
attempts are retained separately. Runtime status and walking verification now
record human PASS. Generated binaries and raw runtime evidence remain ignored.

MATLAB must use the same compiler environment for generated AD libraries.
Deployment metadata records version/ABI; initialization checks PATH and derives
`MW_MINGW64_LOC`. AD caches are separated by compiler major and CRT. Validation
used a fresh MATLAB process to avoid reusing already loaded GCC runtime DLLs.
Future MATLAB sessions should also start with the intended UCRT64 compiler on
PATH; the global Windows PATH and MATLAB compiler preferences were not rewritten.
