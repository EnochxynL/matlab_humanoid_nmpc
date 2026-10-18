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
- `conan/recipes/`: local Coal, Pinocchio, BLASFEO, HPIPM and OCS2 recipes.

OCS2's recipe owns the selected upstream numerical modules and compatibility
adaptations. `cmake/NumericalLibraries.cmake` builds only this project's own
extensions and humanoid modules. The main project consumes OCS2 with
`find_package(ocs2 CONFIG REQUIRED)`.

The recipes fetch fixed upstream Git commits directly. No Git submodule, local
mirror environment variable or separate source lock is required. Sources are
pinned in each recipe's `conandata.yml`; recipe revisions are in `conan.lock`.

## Windows

```powershell
./native/scripts/build_windows.ps1 -Stage All
```

`-Stage Core -WithoutMatlab` builds the standalone controller/replay driver.
`-Stage Platform` builds the AD smoke tests. Both stages first resolve Conan
packages. `-Conan`, `-CMake`, `-Ninja` and `-MatlabRoot` override tool locations.

The retained compiler is MinGW GCC 8.1, x64 POSIX threads and SEH, at
`native/build/windows/toolchains/mingw64`. Archive provenance/checksums are in
`../docs/windows_checkpoint_20261017.md`; the script does not install a compiler.
The project-local Conan cache uses a verified `G:` mapping to
`native/build/windows` to avoid long Boost paths. `-CacheDrive` selects another
free drive. Keep that mapping while using generated CMake files.

Conan builds static BLASFEO, HPIPM and OCS2 modules, then CMake links all AD
users into one controller DLL. Splitting their template-local tape state across
Windows DLLs is unsafe. The recipe retains the Eigen/GCC 8 compatibility overlay,
64-bit generated indices, native process execution and rename-before-load.

Coal 3.0.3 and Pinocchio 4.1.0 recipes derive from Conan Center Index commit
`1c90a5cb2e6353488efefa7d16020c19f9fae0c5` (MIT notice under `conan/recipes/`).
Other retained dependencies include Eigen 3.4, Boost 1.88 and Abseil 20240116.2.
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
