# Dependency and artifact boundaries

The native project consumes OCS2, HPIPM and BLASFEO as Conan packages. Their
local recipes pin the original Git commits in `conandata.yml`. The OCS2 recipe
owns the selected upstream modules and platform compatibility adapters; the
application CMake owns only the humanoid problem, extensions, controller and MEX.

`native/conanfile.txt` declares dependencies and `native/conan.lock` pins recipe
revisions. `native/conan/profiles/` selects the compiler. Local recipes are under
`native/conan/recipes/`. Windows retains GCC 8.1, GENERIC BLASFEO, shared numerical
backend dependencies and static OCS2 modules inside the controller DLL.

The existing `native/third_party/` Git submodules are optional source mirrors and
provenance records, not CMake subdirectories. A recipe can archive the pinned
commit from a local mirror via `G1_RECIPE_SOURCE_ROOT`, or fetch upstream itself.
The archive excludes all working-tree edits, including the preexisting HPIPM
change. No submodule working tree is modified or deleted by this migration.

| Location | Purpose |
| --- | --- |
| `native/build/windows/` | Windows build trees, compiler and Conan cache |
| `native/build/linux/` | Linux build tree and generated Conan files |
| `native/build/cppad_code_gen/` | Runtime-generated C/C++ and AD libraries |
| `install/windows/lib/` | Deployed MEX, controller DLL and runtime dependencies |
| `install/linux/lib/` | Deployed MEX, controller and runtime shared libraries |
| `build/matlab/` | Simulink caches and temporary MATLAB test functions |
| `runtime/evidence/` | Historical validation, fixtures and replay evidence |
| `runtime/walking_*/` | Walking logs and visual validation evidence |

MATLAB uses the deployment directory, never a CMake build directory.
`G1_AD_CACHE_ROOT` can override the AD cache. Old build trees are retained under
`native/build/legacy/` or `native/build/windows/legacy-*`; absolute CMake caches
inside these historical trees are not reusable after relocation. Historical
reports preserve their original paths and timestamps.

Windows: `./native/scripts/build_windows.ps1 -Stage All`.
Linux: `bash native/scripts/build_mex.sh` (or `build_core.sh` without MATLAB).
Linux requires an explicitly prepared Conan profile; `G1_CONAN_HOST_PROFILE`
and `G1_CONAN_BUILD_PROFILE` select it. Windows uses the committed `mingw81`
profile and a short `G:` cache mapping to avoid Boost path-length failures.

## Validation on Windows, 2026-10-18

- All three new recipes built from their original pinned commits. Independent
  OCS2 packaging exposed a missing Coal public-header dependency, now declared.
- Existing dependency versions and recipe revisions are unchanged. Only the
  three newly packaged native libraries were added to the lockfile.
- The complete Windows build/install script passed. Platform AD creation/reload
  passed 2/2 tests; the real OCS2 test returned value 15, Jacobian 8, Hessian 2.
- Deployment imports resolved for 34 binaries. MEX/replay compile commands do
  not include private controller source directories. MATLAB process maps confirm
  loading both project DLLs from `install/windows/lib/`.
- Native replay and actual Simulink planner replay each completed 2401 ticks and
  289 updates. Their CSVs are byte-identical to the preceding Windows migration:
  native SHA-256 `eb2e020ad4d4dc5da8cd89e2053500d14fca9d199520dfe2405bed06d15b7841`,
  model SHA-256 `05c235281eb9543003cfe62e1304f6f72fd6b855fb95f3c4e78283bf544ab3c9`.
  Same-run policy/action identity and stage guards passed.
- The first model attempt stopped when Windows denied renaming the diagnostic
  structural snapshot. Its partial evidence is retained; a fresh-directory retry
  passed completely. The process responsible for the denial was not established.
- PowerShell/Python/shell syntax and Git whitespace checks passed. Linux scripts
  were updated and syntax-checked; no Linux build or new physical walking run was
  performed for this dependency/artifact migration.

Evidence is in `runtime/evidence/conan_artifact_migration/`: `build.log`,
`conan-graph.json`, `source_audit.json`, `lock_comparison.json`,
`include_boundary.json`, `replay_comparison.json`, `core_replay/`, and
`retry/model_replay/acceptance.json`. Historical walking evidence is unchanged.
