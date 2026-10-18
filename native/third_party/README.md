# Optional pinned source mirrors

These Git submodules preserve upstream provenance and optionally accelerate
Conan source acquisition. The application no longer builds their checkouts with
`add_subdirectory()`. Local recipes live in `../conan/recipes/` and pin the same
commits in `conandata.yml`; package revisions are locked in `../conan.lock`.

When `G1_RECIPE_SOURCE_ROOT` points here, recipes archive the pinned commit,
excluding tracked edits and untracked files. Otherwise they fetch upstream.
Initialize mirrors with `git submodule update --init --recursive` when wanted.
No ROS/catkin packages are built. OCS2's Conan package supplies its retained
CppAD/CppADCodeGen headers and platform adaptations.

`dependencies.lock.json` and `../scripts/verify_dependencies.py` audit the mirror
checkouts. That audit remains strict about modifications; packaging committed
files does not erase or approve existing local edits.
