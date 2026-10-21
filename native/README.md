# Native OCS2 problems and MATLAB bindings

This CMake subproject owns the controller, humanoid problem, MEX adapters and
Conan recipes. Run the commands below from the repository root. Runtime robot
assets and configuration stay in the application root.

## Source and dependency boundaries

The optional centroidal OCP is enabled with `-DG1_BUILD_CENTROIDAL=ON`.
Build `g1_centroidal_smoke` with `G1_BUILD_TESTS=ON` for a native construction,
SQP solve and MRT query check. Its configuration is in `config/centroidal/`;
it does not use the whole-body controller API or inverse-dynamics MEX.

The MATLAB/Simulink runtime now uses `ocs2_binding_mex` through
`ocs2.Problem` and `g1.PlannerBackend`. MATLAB owns the tick/stage protocol,
reference validation, first-reset targets, policy caching and consumption checks.
Native bindings own problem, SQP/MRT and optional native diagnostic objects.
See [binding migration](../docs/ocs2_matlab_binding.md).

Both formulations use the shared MATLAB runtime (reference/policy layouts: 428/95 and 359/72).
The optional centroidal formulation also has a complete Simulink walking chain.
See [centroidal Simulink integration](../docs/centroidal_simulink.md).

- `src/g1_controller/model/`: shared robot model and numerical functions.
- `src/g1_controller/problem/`: planning problem, costs, constraints and references.
- `src/g1_controller_mex/InverseDynamicsMex.cpp`: independent model ownership and
  inverse-dynamics binding, linked directly to `humanoid_model` without a facade DLL.
- `src/g1_controller/controller/`: native diagnostic computations.
- `src/ocs2_binding_mex/`: direct OCS2 binding.
- `src/g1_controller_mex/`: inverse-dynamics adapter.
- `conanfile.txt`, `conan.lock`: dependency declarations and recipe revisions.
- `conan/profiles/`: compiler configurations.
- `conan/recipes/`: local Coal, Pinocchio, URDFDOM, BLASFEO, HPIPM and OCS2 recipes.

## Runtime ownership

MATLAB selects task, reference, URDF and log paths through
`g1.plannerConfiguration(root, logDirectory)`. Initialize with
`g1.plannerMex('init', config)`; the old `init(root, folder)` signature is retired.
`run_walking` supplies this configuration automatically.

Both planner models have one `PlannerStep` block with explicit observation,
reference, time identity and solve-request inputs. `MPCSchedule` owns deadlines,
using `[tick; canonical time; raw time]` and the configured `mpc_period` (read once).

`g1.plannerMex('step', observation, reference, identity, request)` submits the
current observation, optionally sets references and solves/installs a policy,
then queries that policy at `time + 0.005`. There are no Install blocks or tokens.
Repeated identical steps reuse their result; changing inputs at the same tick
is rejected. Solver failures require close/init; reference input errors are
checked before invoking the solver. Independent inverse dynamics follows the
returned policy through an ordinary data connection.

The backend decodes references only for requested solves and checks target/event
times, swing settings and current tick. OCS2 keeps its original switched-model
reference manager. Native bindings and numerical problem definitions are unchanged.

The independent `InverseDynamics` model owns its Pinocchio instance and computes
torque without MPC or MRT. OCS2 numerical parallelism follows solver settings.

Application diagnostics are optional. `config.diagnosticsEnabled`
defaults to true for existing acceptance tooling. MATLAB can use
`g1.plannerConfiguration(root, logDirectory, false)` to disable them. With that
setting the backend creates no application log directory, diagnostic OCP clone,
diagnostic observers or diagnostic files. OCS2's own logging and AD code caches
are separate from this switch. `run_walking` keeps diagnostics enabled because
its acceptance checks consume the logs.

`matlab/+g1/plannerMex.m` converts ticks to canonical time and validates raw time.
`g1.PlannerBackend` checks the initial tick and tick continuity; the binding receives
explicit time/state/input/mode arguments. References match observations by tick.
MATLAB also writes the `time_identity` diagnostic table.
`controller/diagnostics/Diagnostics.h` owns native diagnostic objects and typed
in-memory batches, including extra structural evaluations. It creates no files.
Native policy, solver and configuration diagnostics stay beside their C++ objects.
They share the batch drain; MATLAB does not fetch full trajectories or individual
solver properties just to assemble logs. Diagnostic tick metadata is sent only
before requested solves. Application protocol diagnostics remain in MATLAB.
`ocs2.Problem.diagnostics_drain()` drains these batches. Consumers must drain regularly: exceeding
250,000 buffered rows raises an explicit error instead of dropping data.

`reference`, `gait_queries`, `timing_trace`, the empty `logging_load` schema and
`adapter_consumption` are now assembled in MATLAB. The binding does not emit
these tables. `timing_trace` uses session-relative MATLAB `tic/toc`; its clock
origin and scope are declared in `diagnostic_clock.json`. Native solver and
policy-query wall timestamps retain their separate steady-clock origin.

`matlab/+g1/plannerMex.m` initializes and closes both MEX instances. Its
`inverse_dynamics(policy95)` operation calls
`g1_inverse_dynamics_mex('compute', state58, input35)`. MATLAB checks the
current policy identity, caches completed actions and assembles full
`planner_samples` rows. Torque is never sent back to OCS2. The native
planning stage completes at policy query, where MRT diagnostics are captured.
No native handles are passed between MEX files. The Simulink block interfaces
and model files are unchanged. Direct users can initialize the inverse dynamics
MEX alone with a struct containing `taskFile` and `urdfFile`.

The planning and inverse-dynamics MEX link different project libraries. Both
consume `humanoid_model`; `problem` and `dynamics` never depend on each other.
See [native layer boundaries](../docs/native_layers.md) and run
`python native/scripts/verify_layers.py` to check includes.

The wrapper drains after each completed action.
`DiagnosticWriter` owns CSV/metadata/snapshot files. Use the wrapper for normal
MATLAB/Simulink operation, and call `g1.plannerMex('flush')` before inspecting
live files. Close drains the final batch and closes all MATLAB file handles.
Runtime stage, tick, reference and policy identity checks stay enabled in both
modes. See [MATLAB diagnostic ownership](../docs/matlab_diagnostics.md).

Timing trace update timestamps now identify the actual externally requested
observation time, rather than a separately reconstructed nominal deadline.
The trace schema is `event,control_time_s,wall_monotonic_time_s`. The obsolete
`update_eligible` event, duplicate scheduled time and request counter are removed.
See [synchronous backend validation](../docs/synchronous_ocs2_backend.md).

OCS2's recipe owns the selected upstream numerical modules and compatibility
adaptations. `cmake/NumericalLibraries.cmake` builds only this project's own
extensions and humanoid modules. The main project consumes OCS2 with
`find_package(ocs2 CONFIG REQUIRED)`.

The recipes fetch fixed upstream Git commits or checksum-verified release
archives. No Git submodule, local mirror environment variable or separate source
lock is required. Sources are
pinned in each recipe's `conandata.yml`; recipe revisions are in `conan.lock`.

## Windows

```powershell
./native/scripts/build_windows.ps1 -Stage All
```

`-Stage Core -WithoutMatlab` builds the standalone controller/replay driver.
`-Stage Platform` builds the AD smoke tests. Both stages first resolve Conan
packages. `-Conan`, `-CMake`, `-Ninja` and `-MatlabRoot` override tool locations.

Run from an environment with native MSYS2 UCRT64 `gcc`, `g++` and `objdump`
on `PATH` (for example, launch PowerShell from the UCRT64 terminal). The script
checks the compiler target and UCRT headers and detects the GCC version; compiler
paths are not stored in project configuration. GCC is installed and updated by
MSYS2, not downloaded or maintained by this project.

The host profile `conan/profiles/gcc-ucrt64` includes the PATH-based
`gcc` profile. UCRT is included in package IDs to separate its binaries from
other CRTs. Tool packages use the build profile without that host ABI marker.
Builds use `native/build/windows/gcc<major>-ucrt64/`, with separate core, dependency
and platform test directories. `-InstallPrefix` selects a trial deployment;
the normal default remains `install/windows/`.
The project-local Conan cache uses a verified `G:` mapping to
`native/build/windows` to avoid long Boost paths. `-CacheDrive` selects another
free drive. Keep that mapping while using generated CMake files.

Conan builds static BLASFEO, HPIPM and OCS2 modules, then CMake links all AD
users into one controller DLL. Splitting their template-local tape state across
Windows DLLs is unsafe. The recipe retains the Eigen/GCC 8 compatibility overlay,
64-bit generated indices, native process execution and rename-before-load.

Coal 3.0.3 and Pinocchio 4.1.0 recipes derive from Conan Center Index commit
`1c90a5cb2e6353488efefa7d16020c19f9fae0c5` (MIT notice under `conan/recipes/`).
URDFDOM 4.0.0 derives from locked ConanCenter revision
`e68d7aed7dbdd349a77e7430fe7f1f8c`, with explicit `<cstdint>` includes for GCC 16.
Other retained dependencies include Eigen 3.4 and Boost 1.88.
Abseil was unused by the retained targets and is no longer a root dependency.
`G1_PINOCCHIO_ARCHIVE` can supply a previously downloaded, checksum-verified
release archive.

## Linux

```bash
bash native/scripts/build_mex.sh
# Or without MATLAB:
bash native/scripts/build_core.sh
```

Prepare a Conan compiler profile first. `G1_CONAN_HOST_PROFILE` and
`G1_CONAN_BUILD_PROFILE` select profiles; both default to `default`.
`G1_MATLAB_ROOT` selects MATLAB. Linux builds use Conan too, without a ROS prefix.
Linux execution must be validated on Linux; Windows checks do not establish it.

## Generated files and deployment

Build trees, tools and AD caches live under `native/build/`. Successful builds
install MATLAB binaries and runtime libraries into `install/windows/lib/` or
`install/linux/lib/` at the application root. MATLAB initializes from that
installation, not from a build tree. Simulink caches live in `build/matlab/`.
Validation logs and fixtures live in `runtime/evidence/`; walking runs remain in
`runtime/walking_*/`. Old moved CMake caches are historical snapshots, not valid
build trees. See `../docs/dependency_artifact_layout.md`.

Floating-point roundoff is not a stopping gate. Same-run protocol identities,
2401 control ticks, 289 updates and duplicate-step rejection checks remain required.
Historical walking passes are recorded in `../docs/windows_checkpoint_20261017.md`
and `../docs/native_matlab_adapters.md`.

MATLAB must be launched with the same compiler environment on PATH. Deployed
`compiler-info.json` records the compiler version and ABI, not its install path.
Initialization verifies the compiler and derives `MW_MINGW64_LOC` from PATH for
MATLAB's compiler integration. `G1_RUNTIME_DIRECTORY` selects a trial runtime;
AD and Simulink caches are separated by compiler/ABI, and CppAD invokes `gcc` from PATH.
An explicit external `G1_AD_CACHE_ROOT` override is still honored.
