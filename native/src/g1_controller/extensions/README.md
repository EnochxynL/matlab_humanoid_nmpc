# Application-owned OCP extensions

These files belong to `humanoid::ocp_extensions`, not the official `ocs2`
namespace. Geometry helpers use its `geometry` child namespace; augmented
penalty interfaces use its `augmented` child namespace. Files and types are
named by capability because these implementations contain no G1 joint names,
robot dimensions or G1-specific parameter choices.

The extensions are compiled as `humanoid_ocp_extensions`. Official OCS2 source
and headers remain unmodified. The public controller API does not expose these
types; application consumers explicitly qualify them to distinguish them from
similarly named official classes.

- `PieceWisePolynomialBarrierPenalty`: preserved application polynomial formula;
  this is not an alias to the official implementation.
- `PenaltyBase`, `augmented::AugmentedPenaltyBase`: application parameter
  metadata interfaces derived from their explicitly qualified official bases.
- `RelaxedBarrierPenalty`: preserved scalar formula and configuration metadata.
- `MultidimensionalPenalty`, `StateSoftConstraint`, `StateInputSoftConstraint`:
  preserved reduction, owning-cost and chain-rule implementations.
- `QuadraticStateCost`, `QuadraticStateInputCost`: metadata subclasses;
  numerical evaluation remains in the official quadratic cost base methods.
- `ActiveConstraint`, `MeasuredModeReceiver`: application interfaces for
  constraint activation and measured contact-mode propagation.
- `TermRegistry`: application term names mapped to official collection indices.
- `EndEffectorKinematics`, `geometry` in `RotationTransforms`: retained 6D
  interfaces and rotation math with no robot-specific assumptions.
- `PinocchioEndEffectorKinematicsCppAd`: the matching 6D implementation from
  the same pinned OCS2 fork, built as `humanoid_pinocchio_kinematics` when the
  centroidal OCP is enabled. It uses generic state/input mappings and does not
  depend on the whole-body OCP or inverse-dynamics facade.

The penalty and owning-cost implementations originated at manumerous/ocs2_ros2
commit bf9ae46f3c6dc751348d8002c03aa2f4a7cc17e7. Their BSD notices remain in the
source files. This naming change does not alter formulas, parameters, operation
order, constraints or solver settings. Full fixed-input replay remains the
acceptance criterion; this does not establish a new physical walking result.
