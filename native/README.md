# Native OCS2 controller and MATLAB MEX

This CMake subproject owns the controller, humanoid problem, Planner MEX and
Conan recipes. Run the commands below from the repository root. Runtime robot
assets and configuration stay in the application root.

## Source and dependency boundaries

- `include/g1_controller/G1Controller.h`: public controller and data types.
- `src/g1_controller/`: controller implementation, extensions and humanoid OCP.
- `src/g1_controller_mex/`: MATLAB adapter.
- `conanfile.txt`, `conan.lock`: dependency declarations and recipe revisions.
- `conan/profiles/`: compiler configurations.
- `conan/recipes/`: local Coal, Pinocchio, URDFDOM, BLASFEO, HPIPM and OCS2 recipes.

## Runtime ownership

MATLAB selects task, reference, URDF and log paths through
`g1.plannerConfiguration(root, logDirectory)`. Initialize with
`g1_ocp_mex('init', config)`; the old `init(root, folder)` signature is retired.
`run_walking` supplies this configuration automatically.

The `g1_planner/MPCSchedule` MATLAB Function block owns update deadlines.
`update_mpc(token, request)` solves synchronously only when requested; otherwise
it prepares the observation for querying the cached policy. The first update
must request a solve. Duplicate updates must retain the same request and do not
solve twice. C++ checks stage and policy identities, not external scheduling
eligibility. A failed solve requires closing and initializing a fresh backend.

`PlannerContext` only assembles and owns the native OCP, MPC and diagnostic
objects. `Ocs2Backend` owns MRT access, synchronous solve/policy installation,
policy evaluation and inverse dynamics. There is no application solver worker,
condition variable, duplicate 60 Hz clock or five-second worker wait timeout.
OCS2's own numerical parallelism remains configured by its solver settings.
Whole policies stay native; MATLAB receives only the evaluated policy sample.

Timing trace update timestamps now identify the actual externally requested
observation time, rather than a separately reconstructed nominal deadline.
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
2401 control ticks, 289 updates and stage rejection checks remain required.
Historical walking passes are recorded in `../docs/windows_checkpoint_20261017.md`
and `../docs/native_matlab_adapters.md`.

MATLAB must be launched with the same compiler environment on PATH. Deployed
`compiler-info.json` records the compiler version and ABI, not its install path.
Initialization verifies the compiler and derives `MW_MINGW64_LOC` from PATH for
MATLAB's compiler integration. `G1_RUNTIME_DIRECTORY` selects a trial runtime;
AD and Simulink caches are separated by compiler/ABI, and CppAD invokes `gcc` from PATH.
An explicit external `G1_AD_CACHE_ROOT` override is still honored.
