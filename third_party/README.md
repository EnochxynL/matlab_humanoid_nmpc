# Numerical dependencies

OCS2, BLASFEO and HPIPM are unmodified, commit-pinned Git submodules.
Initialize after cloning with `git submodule update --init --recursive`.
The parent repository tracks gitlinks, not copies of the vendor source files.
Versions and origins are recorded in `dependencies.lock.json`.

OCS2 is the official https://github.com/leggedrobotics/ocs2 repository.
Its ROS/catkin packages belong to upstream; the project does not build them.
`cmake/NumericalLibraries.cmake` selects numerical libraries with plain CMake.
Pinocchio, Boost, Eigen, Abseil and URDFDOM are discovered as installed packages.
CppAD/CppADCodeGen headers currently come from the pinned OCS2 thirdparty tree.
No ROS communication or MATLAB SDK is needed by the controller core.
