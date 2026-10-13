# G1 application extensions for unmodified official OCS2

These classes are owned and compiled by this application; no official source or
header is shadowed. Prefix `G1` distinguishes them from official OCS2 types.
The public controller API does not expose these classes.

- `G1PieceWisePolynomialBarrierPenalty`: original G1 polynomial definition,
  implementing the official penalty interface.
- `G1PenaltyBase`, `G1AugmentedPenaltyBase`: application parameter metadata
  interfaces derived from official penalty interfaces.
- `G1RelaxedBarrierPenalty`: preserved scalar formula and configuration metadata.
- `G1MultidimensionalPenalty`, `G1StateSoftConstraint`,
  `G1StateInputSoftConstraint`: application owning-cost implementations, retaining
  the previous reduction and chain rule while permitting generated mathematics
  to be installed. They implement official cost interfaces.
- `G1QuadraticStateCost`, `G1QuadraticStateInputCost`: metadata-only subclasses;
  values and derivatives are evaluated by the official quadratic cost methods.
- `ActiveConstraint`: application activation interface. Its default behavior for
  an official constraint without that interface matches the former base methods.
- `TermRegistry`: records names when application terms are constructed, and uses
  official `getTermIndex` to recover each collection's current ordering.
- `G1EndEffectorKinematics`, `G1RotationTransforms`: original 6D application
  interfaces and rotation math, separated from the official library.
- `MeasuredModeReceiver`: conveys observation mode to application reference
  updates without changing official MPC/SQP signatures.

The preserved penalty and owning-cost implementations originated at
manumerous/ocs2_ros2 commit bf9ae46f3c6dc751348d8002c03aa2f4a7cc17e7.
Their BSD copyright/license notices remain in each extracted file. This is an
explicit application extension, not an allegedly unmodified official class.
No cost weight, constraint, formula, solver iteration limit or physical model
was intentionally changed. Full output replay is the acceptance criterion.
