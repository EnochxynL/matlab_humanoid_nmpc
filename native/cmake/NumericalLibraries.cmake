set(G1_NUMERICAL_LIBRARY_TYPE STATIC)
file(GLOB humanoid_extension_sources CONFIGURE_DEPENDS "${G1_CORE_DIR}/extensions/*.cpp")
add_library(humanoid_ocp_extensions ${G1_NUMERICAL_LIBRARY_TYPE} ${humanoid_extension_sources})
target_include_directories(humanoid_ocp_extensions PUBLIC "${G1_CORE_DIR}")
target_link_libraries(humanoid_ocp_extensions PUBLIC ocs2_core ocs2_robotic_tools)
add_library(humanoid_common_mpc ${G1_NUMERICAL_LIBRARY_TYPE}
  "${G1_CORE_DIR}/problem/humanoid_common_mpc/src/contact/ContactCenterPoint.cpp"
  "${G1_CORE_DIR}/problem/humanoid_common_mpc/src/contact/ContactPolygon.cpp"
  "${G1_CORE_DIR}/problem/humanoid_common_mpc/src/contact/ContactRectangle.cpp"
  "${G1_CORE_DIR}/problem/humanoid_common_mpc/src/pinocchio_model/pinocchioUtils.cpp"
  "${G1_CORE_DIR}/problem/humanoid_common_mpc/src/pinocchio_model/createPinocchioModel.cpp"
  "${G1_CORE_DIR}/problem/humanoid_common_mpc/src/pinocchio_model/DynamicsHelperFunctions.cpp"
  "${G1_CORE_DIR}/problem/humanoid_common_mpc/src/common/ModelSettings.cpp"
  "${G1_CORE_DIR}/problem/humanoid_common_mpc/src/constraint/JointLimitsSoftConstraint.cpp"
  "${G1_CORE_DIR}/problem/humanoid_common_mpc/src/constraint/FootCollisionConstraint.cpp"
  "${G1_CORE_DIR}/problem/humanoid_common_mpc/src/constraint/ZeroWrenchConstraint.cpp"
  "${G1_CORE_DIR}/problem/humanoid_common_mpc/src/constraint/EndEffectorKinematicsTwistConstraint.cpp"
  "${G1_CORE_DIR}/problem/humanoid_common_mpc/src/constraint/FrictionForceConeConstraint.cpp"
  "${G1_CORE_DIR}/problem/humanoid_common_mpc/src/constraint/ContactMomentXYConstraintCppAd.cpp"
  "${G1_CORE_DIR}/problem/humanoid_common_mpc/src/constraint/EndEffectorKinematicsLinearVelConstraint.cpp"
  "${G1_CORE_DIR}/problem/humanoid_common_mpc/src/cost/StateInputQuadraticCost.cpp"
  "${G1_CORE_DIR}/problem/humanoid_common_mpc/src/cost/EndEffectorKinematicCostHelpers.cpp"
  "${G1_CORE_DIR}/problem/humanoid_common_mpc/src/cost/EndEffectorKinematicsQuadraticCost.cpp"
  "${G1_CORE_DIR}/problem/humanoid_common_mpc/src/cost/ExternalTorqueQuadraticCostAD.cpp"
  "${G1_CORE_DIR}/problem/humanoid_common_mpc/src/gait/Gait.cpp"
  "${G1_CORE_DIR}/problem/humanoid_common_mpc/src/gait/GaitSchedule.cpp"
  "${G1_CORE_DIR}/problem/humanoid_common_mpc/src/gait/ModeSequenceTemplate.cpp"
  "${G1_CORE_DIR}/problem/humanoid_common_mpc/src/swing_foot_planner/CubicSpline.cpp"
  "${G1_CORE_DIR}/problem/humanoid_common_mpc/src/swing_foot_planner/SplineCpg.cpp"
  "${G1_CORE_DIR}/problem/humanoid_common_mpc/src/swing_foot_planner/SwingTrajectoryPlanner.cpp"
  "${G1_CORE_DIR}/problem/humanoid_common_mpc/src/reference_manager/SwitchedModelReferenceManager.cpp"
  "${G1_CORE_DIR}/problem/humanoid_common_mpc/src/HumanoidCostConstraintFactory.cpp"
  "${G1_CORE_DIR}/problem/humanoid_common_mpc/src/HumanoidPreComputation.cpp"
  "${G1_CORE_DIR}/problem/humanoid_common_mpc/src/initialization/WeightCompInitializer.cpp"
)
target_include_directories(humanoid_common_mpc PUBLIC "${G1_CORE_DIR}/problem/humanoid_common_mpc/include" "${G1_CORE_DIR}" "${G1_CORE_DIR}/problem")
target_link_libraries(humanoid_common_mpc PUBLIC humanoid_ocp_extensions ocs2_core ocs2_mpc ocs2_pinocchio_interface pinocchio::pinocchio)
target_compile_options(humanoid_common_mpc PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wno-ignored-attributes>)

add_library(humanoid_wb_mpc ${G1_NUMERICAL_LIBRARY_TYPE}
  "${G1_CORE_DIR}/problem/humanoid_wb_mpc/src/dynamics/DynamicsHelperFunctions.cpp"
  "${G1_CORE_DIR}/problem/humanoid_wb_mpc/src/dynamics/WBAccelDynamicsAD.cpp"
  "${G1_CORE_DIR}/problem/humanoid_wb_mpc/src/end_effector/PinocchioEndEffectorDynamicsCppAd.cpp"
  "${G1_CORE_DIR}/problem/humanoid_wb_mpc/src/constraint/EndEffectorDynamicsLinearAccConstraint.cpp"
  "${G1_CORE_DIR}/problem/humanoid_wb_mpc/src/constraint/EndEffectorDynamicsAccelerationsConstraint.cpp"
  "${G1_CORE_DIR}/problem/humanoid_wb_mpc/src/constraint/ZeroAccelerationConstraintCppAd.cpp"
  "${G1_CORE_DIR}/problem/humanoid_wb_mpc/src/constraint/SwingLegVerticalConstraintCppAd.cpp"
  "${G1_CORE_DIR}/problem/humanoid_wb_mpc/src/constraint/JointMimicDynamicsConstraint.cpp"
  "${G1_CORE_DIR}/problem/humanoid_wb_mpc/src/WBMpcInterface.cpp"
  "${G1_CORE_DIR}/problem/humanoid_wb_mpc/src/WBMpcPreComputation.cpp"
  "${G1_CORE_DIR}/problem/humanoid_wb_mpc/src/cost/EndEffectorDynamicsCostHelpers.cpp"
  "${G1_CORE_DIR}/problem/humanoid_wb_mpc/src/cost/EndEffectorDynamicsQuadraticCost.cpp"
  "${G1_CORE_DIR}/problem/humanoid_wb_mpc/src/cost/EndEffectorDynamicsFootCost.cpp"
  "${G1_CORE_DIR}/problem/humanoid_wb_mpc/src/cost/JointTorqueCostCppAd.cpp"
)
target_include_directories(humanoid_wb_mpc PUBLIC "${G1_CORE_DIR}/problem/humanoid_wb_mpc/include" "${G1_CORE_DIR}" "${G1_CORE_DIR}/problem")
target_link_libraries(humanoid_wb_mpc PUBLIC humanoid_common_mpc ocs2_sqp ocs2_ddp ocs2_centroidal_model)
target_compile_options(humanoid_wb_mpc PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wno-ignored-attributes>)
