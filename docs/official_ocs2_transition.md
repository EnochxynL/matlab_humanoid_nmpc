# Official OCS2 transition — acceptance PASS

The source root remains `cpp/`. The independent target is `g1_controller_core`,
producing `libg1_controller.so`; the MATLAB adapter produces `g1_ocp_mex.mexa64`.

## Dependency boundary

OCS2 is the official leggedrobotics/ocs2 repository at
`26386754b8bf31ab78b503971d0cbf4fdbcd7cb4`. OCS2, BLASFEO and HPIPM are complete,
unmodified Git submodules with fixed gitlinks; origins and versions are in
`third_party/dependencies.lock.json`. Initialize with
`git submodule update --init --recursive` and verify with
`python3 scripts/verify_dependencies.py`.

The plain-CMake wrapper selects numerical targets and does not execute upstream
ROS/catkin packaging. Pinocchio and other system numerical packages are package-
discovered. CppAD/CppADCodeGen headers remain supplied by the pinned OCS2 tree.

Two explicit build compatibility adaptations are required on this machine:

1. With Pinocchio 3, generate two compilation units from official sources with
   exactly three Frame `parent` accessors renamed `parentJoint`. One accessor
   prints model information; two belong to the retained centroidal conversion
   library. This matches the field used by the previous accepted build. The
   submodule files remain unchanged; details are in
   `cmake/PinocchioCompatibility.cmake`.
2. Supply standard `<fstream>` through the SQP target's compiler include option,
   because the pinned official SqpSolver.cpp uses `ofstream` without including it.

These are documented external build adaptations, not claims that the compiled
translation units are wholly byte-identical to upstream. No SQP calculation,
OCP formula, weight, constraint or physical model was changed.

## G1 application extensions

`cpp/extensions/` owns the polynomial penalty, parameter interfaces, soft-cost
ownership and unchanged reduction/chain rule, activation and term metadata,
6D end-effector interface, and preserved rotation utilities. The extracted
implementations retain their original license notices and fork provenance.
Quadratic cost values and derivatives call official OCS2 methods; the application
subclasses merely retain constructor weights for the accepted generated math.

Observation mode is passed to an application reference-manager interface before
solving. Official MPC/SQP signatures remain unchanged. The original ground-height
calculation and subsequent zero-height assignment are preserved. Read-only
observation copies replace the fork's MRT getter. AD cost callbacks conform to
official const signatures using mutable per-object Pinocchio caches.

Fork-only internal timing getters are unavailable. Their diagnostic columns are
NaN/UNKNOWN and labelled as unavailable, never substituted with invented timings.
Official aggregate benchmark output remains available.

## Acceptance

- Linux core and MEX build: PASS.
- MATLAB SDK discovery disabled, core target built using the same incremental
  cache: PASS; all 427 compile units in that configuration have no MATLAB SDK
  or MATLAB adapter paths.
- Standalone split query/ID: 2401 ticks, max error 0, differing double bits 0.
- Standalone combined step: 2401 ticks, max error 0, differing double bits 0.
- Fresh MATLAB-process MEX replay: 2401 ticks, 289 MPC updates, all 118 output
  doubles exactly equal to the accepted 4.8-second walking observation replay.
- Split/combined MEX identity and invalid/stale policy rejection checks: PASS.
- Models and generated mathematics: original SHA values unchanged. Configuration
  and Simulink models were not edited.
- Core dynamic dependencies: no MATLAB or ROS middleware; no missing library.

Machine-readable evidence: `docs/core_extraction_acceptance.json`.
Detailed build/replay logs and old vendor copies remain under ignored
`build/core_extraction/`. No new physical simulation or visual acceptance was
performed; the tests replay saved real observations and references.

The existing desktop MATLAB process still had old libraries loaded during this
work. Restart it before using the new MEX, so old and new library ABIs cannot mix.
The fresh-process MEX regression passed; the user's desktop was not terminated.
Windows adaptation is outside this phase and was not performed.
