# Native OCS2 subproject

This directory owns the C++ controller, Planner MEX, pinned submodules, CMake
adaptations, Conan manifests/recipes and native build tests. Run commands below
from the repository root. Runtime config/assets remain in the application root;
`G1Controller::initialize` still receives that application root explicitly.

The public C++ interface is `cpp/include/g1_controller/G1Controller.h`, containing
both the controller declaration and its standard C++ data types. Consumers include
`g1_controller/G1Controller.h`; `cpp/controller/` contains private implementation.

Linux builds: `bash native/scripts/build_mex.sh` (MEX) or
`bash native/scripts/build_core.sh` (core only), output in `build/native-linux/lib`.

## Windows build

The current configuration uses native MinGW GCC 8.1 (x64, POSIX threads, SEH) and
MATLAB R2025b, with Conan 2 for C++ dependencies. It does not mix MSYS2 prebuilt
C++ libraries into this toolchain. OCS2, BLASFEO and HPIPM remain pinned submodules.

`native/scripts/build_windows.ps1 -Stage All` builds the platform smoke tests and core.
`-Stage Core -WithoutMatlab` builds the standalone core/replay
driver. Run PowerShell with the installed development tools available, or pass
`-Conan`, `-CMake`, `-Ninja` and `-MatlabRoot` explicitly.

The script expects the isolated compiler at `build/windows/toolchains/mingw64`.
Archive provenance and checksums are recorded in
`../docs/windows_checkpoint_20261017.md`; it does not install another compiler.

The build uses CMake 3.31 where available. A verified `G:` subst mapping to
`build/windows` shortens Conan source paths. Change `-CacheDrive` if that drive is
already occupied; an unrelated mapping is never replaced. The mapping is kept
because generated CMake package files refer to it. Re-running the script verifies
or recreates it. Remove it with `subst G: /d` only after builds are no longer using it.
The Conan cache is project-local, not the user's default cache.

Local Pinocchio 4.1.0 and Coal 3.0.3 recipes are based on Conan Center Index commit
`1c90a5cb2e6353488efefa7d16020c19f9fae0c5`; the MIT notice is in
`conan/recipes/LICENSE.conan-center-index`. Core dependencies currently use Eigen 3.4.0,
Boost 1.88.0 and Abseil 20240116.2. These are Windows build choices, not a claim of
an exact Linux package replica. A real core lockfile is saved only after successful
resolution/build. The three stateless adapters are now native MATLAB functions;
only `g1_ocp_mex` is built for MATLAB. Conversion implementations now live inside
the five MATLAB Function blocks of `g1_planner.slx`, without separate .m kernels.

For an existing Pinocchio release archive, set `G1_PINOCCHIO_ARCHIVE` to its
absolute path before building. The recipe verifies the same pinned SHA-256 used
for the upstream download before extraction; an unmatched archive is rejected.

Windows platform adaptations are generated into the build directory by
`cmake/WindowsCompatibility.cmake`; upstream submodule source remains unchanged.
They cover native compiler process execution, DLL loading, 64-bit generated-index
ABI, and rename-before-load behavior. No AD expressions are replaced. Native
generation/reload tests include sparse Jacobian/Hessian checks and paths with spaces.
The GCC 8 compatibility overlay also restricts Eigen 3.4's empty SSE register
barrier to vectorizable packet types: GCC 8 otherwise crashes while compiling
an AD scalar matrix product. Matrix arithmetic and double-vector packet paths
are preserved. Conan's Eigen package itself is not modified. A related upstream
custom-scalar register-constraint report is
https://gitlab.com/libeigen/eigen/-/issues/3059 (a different Eigen/compiler version).

`g1.initialize` selects the Windows runtime directory and modifies only the current
MATLAB process's DLL search PATH. Runtime DLLs are deployed from host dependencies
with a provenance manifest. Windows AD caches are separate from Linux caches.
Application environment reads use Windows process APIs because MinGW CRT `getenv`
does not reflect MATLAB `setenv` in the tested session.

On Windows the internal OCS2/humanoid numerical targets are static libraries
linked into `g1_controller.dll`. CppAD recording state uses template-local
statics: separate PE DLLs otherwise record a lambda against different tape
state (the integration probe returned a constant value and zero derivatives).
Keeping these targets in one image preserves their shared tape state; Linux
keeps the original shared-library layout. Pinocchio, Boost and C dependencies
remain separately packaged. This is a link-layout adaptation, not an AD formula change.

The historical bitwise conversion failure is retained. The user subsequently
authorized continuing through roundoff differences and evaluating the walking
result. Run `validate_matlab_adapters` after adding `tests` to the MATLAB path to
check the actual conversion block code and report floating-point differences.
Temporary test entry points are generated from the model and removed on exit.
Protocol, dimensions, finite outputs and control-time
identity checks remain active. A completed build does not itself establish that
the controller or physical walking passed.

The planner model replay uses `run_windows_planner_replay` after adding
`tools/windows` to the MATLAB path. It scopes generated files to Windows caches
and invokes `validate_planner_dataflow('windows/planner_dataflow',true)`.
Its baseline numerical differences are
reported rather than blocking the authorized continuation. The model output must
still match the output published by the same runtime, and time/mode identities,
2401 control ticks, 289 MPC updates and stage rejection checks remain mandatory.

Validation completed on 2026-10-17: core/MEX build, AD creation/reload, native
and actual model replay, then `run_windows_walking` with the original 4.8 s plant
run. See `../docs/windows_checkpoint_20261017.md` for measurements and evidence.
The walking run took 656 s, exceeding the MATLAB tool's 600 s call timeout;
MATLAB continued and saved the successful result. Check saved evidence before
repeating a long run. Human animation review was subsequently confirmed PASS by the user.

Fresh Windows builds use `build/windows/native-core` and
`build/windows/native-platform`. Old `build/windows/core` and `platform` caches
refer to the pre-move source tree and are retained only as historical artifacts.
MATLAB initialization selects the new native-core runtime automatically.
The dependency lock paths are relative to this native subproject.
