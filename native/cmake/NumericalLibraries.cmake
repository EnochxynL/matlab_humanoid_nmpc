# Shared model definitions and numerical algorithms. No planning or inverse-dynamics facade.
add_library(humanoid_model STATIC
  "${G1_CORE_DIR}/model/humanoid_common_mpc/src/common/ModelSettings.cpp"
  "${G1_CORE_DIR}/model/humanoid_common_mpc/src/contact/ContactCenterPoint.cpp"
  "${G1_CORE_DIR}/model/humanoid_common_mpc/src/contact/ContactPolygon.cpp"
  "${G1_CORE_DIR}/model/humanoid_common_mpc/src/contact/ContactRectangle.cpp"
  "${G1_CORE_DIR}/model/humanoid_common_mpc/src/pinocchio_model/createPinocchioModel.cpp"
  "${G1_CORE_DIR}/model/humanoid_common_mpc/src/pinocchio_model/DynamicsHelperFunctions.cpp"
  "${G1_CORE_DIR}/model/humanoid_common_mpc/src/pinocchio_model/pinocchioUtils.cpp"
  "${G1_CORE_DIR}/model/humanoid_wb_mpc/src/dynamics/DynamicsHelperFunctions.cpp")
target_include_directories(humanoid_model PUBLIC
  "${G1_CORE_DIR}/model/humanoid_common_mpc/include"
  "${G1_CORE_DIR}/model/humanoid_wb_mpc/include"
  PRIVATE "${G1_CORE_DIR}")
target_link_libraries(humanoid_model PUBLIC
  ocs2_core ocs2_robotic_tools ocs2_pinocchio_interface pinocchio::pinocchio)
target_compile_options(humanoid_model PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wno-ignored-attributes>)

set(G1_NUMERICAL_LIBRARY_TYPE STATIC)
file(GLOB humanoid_extension_sources CONFIGURE_DEPENDS "${G1_CORE_DIR}/extensions/*.cpp")
list(REMOVE_ITEM humanoid_extension_sources "${G1_CORE_DIR}/extensions/PinocchioEndEffectorKinematicsCppAd.cpp")
add_library(humanoid_ocp_extensions ${G1_NUMERICAL_LIBRARY_TYPE} ${humanoid_extension_sources})
target_include_directories(humanoid_ocp_extensions PUBLIC "${G1_CORE_DIR}")
target_link_libraries(humanoid_ocp_extensions PUBLIC ocs2_core ocs2_robotic_tools)
add_library(humanoid_common_mpc ${G1_NUMERICAL_LIBRARY_TYPE}
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
target_link_libraries(humanoid_common_mpc PUBLIC humanoid_model humanoid_ocp_extensions ocs2_mpc)
target_compile_options(humanoid_common_mpc PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wno-ignored-attributes>)

add_library(humanoid_wb_mpc ${G1_NUMERICAL_LIBRARY_TYPE}
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

# Second OCP implementation; no dependency on the whole-body OCP or its facade.
option(G1_BUILD_CENTROIDAL "Build the experimental centroidal OCP and native smoke test" OFF)
if(G1_BUILD_CENTROIDAL)
  add_library(humanoid_pinocchio_kinematics STATIC
    "${G1_CORE_DIR}/extensions/PinocchioEndEffectorKinematicsCppAd.cpp")
  target_link_libraries(humanoid_pinocchio_kinematics PUBLIC
    humanoid_ocp_extensions ocs2_pinocchio_interface pinocchio::pinocchio)
  target_compile_options(humanoid_pinocchio_kinematics PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wno-ignored-attributes>)
  add_library(humanoid_centroidal_model INTERFACE)
  target_include_directories(humanoid_centroidal_model INTERFACE
    "${G1_CORE_DIR}/model/humanoid_centroidal_mpc/include")
  target_link_libraries(humanoid_centroidal_model INTERFACE humanoid_model ocs2_centroidal_model)
  add_library(humanoid_centroidal_mpc STATIC
    "${G1_CORE_DIR}/problem/humanoid_centroidal_mpc/src/CentroidalMpcInterface.cpp"
    "${G1_CORE_DIR}/problem/humanoid_centroidal_mpc/src/dynamics/CentroidalDynamicsAD.cpp"
    "${G1_CORE_DIR}/problem/humanoid_centroidal_mpc/src/cost/CentroidalMpcEndEffectorFootCost.cpp"
    "${G1_CORE_DIR}/problem/humanoid_centroidal_mpc/src/cost/ICPCost.cpp"
    "${G1_CORE_DIR}/problem/humanoid_centroidal_mpc/src/constraint/JointMimicKinematicConstraint.cpp"
    "${G1_CORE_DIR}/problem/humanoid_centroidal_mpc/src/constraint/NormalVelocityConstraintCppAd.cpp"
    "${G1_CORE_DIR}/problem/humanoid_centroidal_mpc/src/constraint/ZeroVelocityConstraintCppAd.cpp"
    "${G1_CORE_DIR}/problem/humanoid_centroidal_mpc/src/initialization/CentroidalWeightCompInitializer.cpp")
  target_include_directories(humanoid_centroidal_mpc PUBLIC
    "${G1_CORE_DIR}/problem/humanoid_centroidal_mpc/include")
  target_link_libraries(humanoid_centroidal_mpc PUBLIC
    humanoid_centroidal_model humanoid_common_mpc humanoid_pinocchio_kinematics ocs2_sqp ocs2_ddp)
  target_compile_options(humanoid_centroidal_mpc PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wno-ignored-attributes>)
endif()
