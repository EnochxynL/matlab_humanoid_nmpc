include("${PROJECT_SOURCE_DIR}/cmake/PinocchioCompatibility.cmake")
file(GLOB g1_extension_sources CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/cpp/extensions/*.cpp")
add_library(g1_ocp_extensions SHARED ${g1_extension_sources})
target_include_directories(g1_ocp_extensions PUBLIC "${PROJECT_SOURCE_DIR}/cpp")
target_link_libraries(g1_ocp_extensions PUBLIC ocs2_core ocs2_robotic_tools)
# Plain-CMake build adapter. Vendored OCS2 sources and original build files remain unchanged.
add_library(ocs2_thirdparty INTERFACE)
target_include_directories(ocs2_thirdparty SYSTEM INTERFACE "${G1_OCS2_SOURCE_DIR}/ocs2_thirdparty/include")
add_library(robot_core INTERFACE)
target_include_directories(robot_core INTERFACE "${G1_CORE_DIR}/robot/robot_core/include")
target_link_libraries(robot_core INTERFACE Eigen3::Eigen)

add_library(ocs2_core SHARED
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/Types.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/augmented_lagrangian/AugmentedLagrangian.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/augmented_lagrangian/StateAugmentedLagrangian.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/augmented_lagrangian/StateInputAugmentedLagrangian.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/augmented_lagrangian/StateAugmentedLagrangianCollection.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/augmented_lagrangian/StateInputAugmentedLagrangianCollection.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/automatic_differentation/CppAdInterface.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/automatic_differentation/CppAdSparsity.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/automatic_differentation/FiniteDifferenceMethods.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/constraint/StateConstraintCppAd.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/constraint/StateInputConstraintCppAd.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/constraint/StateConstraintCollection.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/constraint/StateInputConstraintCollection.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/constraint/LinearStateConstraint.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/constraint/LinearStateInputConstraint.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/control/FeedforwardController.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/control/LinearController.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/control/StateBasedLinearController.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/cost/QuadraticStateCost.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/cost/QuadraticStateInputCost.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/cost/StateCostCollection.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/cost/StateCostCppAd.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/cost/StateInputCostCollection.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/cost/StateInputCostCppAd.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/cost/StateInputGaussNewtonCostAd.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/dynamics/ControlledSystemBase.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/dynamics/LinearSystemDynamics.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/dynamics/SystemDynamicsBase.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/dynamics/SystemDynamicsBaseAD.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/dynamics/SystemDynamicsLinearizer.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/dynamics/TransferFunctionBase.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/integration/SensitivityIntegrator.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/integration/SensitivityIntegratorImpl.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/integration/Integrator.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/integration/IntegratorBase.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/integration/RungeKuttaDormandPrince5.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/integration/OdeBase.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/integration/Observer.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/integration/StateTriggeredEventHandler.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/integration/SystemEventHandler.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/reference/ModeSchedule.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/reference/TargetTrajectories.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/loopshaping/LoopshapingDefinition.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/loopshaping/LoopshapingPropertyTree.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/loopshaping/LoopshapingFilter.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/loopshaping/LoopshapingPreComputation.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/loopshaping/cost/LoopshapingCost.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/loopshaping/cost/LoopshapingStateCost.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/loopshaping/cost/LoopshapingStateInputCost.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/loopshaping/cost/LoopshapingCostEliminatePattern.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/loopshaping/cost/LoopshapingCostOutputPattern.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/loopshaping/soft_constraint/LoopshapingSoftConstraint.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/loopshaping/soft_constraint/LoopshapingStateInputSoftConstraint.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/loopshaping/soft_constraint/LoopshapingSoftConstraintEliminatePattern.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/loopshaping/soft_constraint/LoopshapingSoftConstraintOutputPattern.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/loopshaping/augmented_lagrangian/LoopshapingAugmentedLagrangian.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/loopshaping/augmented_lagrangian/LoopshapingStateAugmentedLagrangian.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/loopshaping/augmented_lagrangian/LoopshapingStateInputAugmentedLagrangian.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/loopshaping/augmented_lagrangian/LoopshapingAugmentedLagrangianEliminatePattern.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/loopshaping/augmented_lagrangian/LoopshapingAugmentedLagrangianOutputPattern.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/loopshaping/constraint/LoopshapingConstraint.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/loopshaping/constraint/LoopshapingStateConstraint.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/loopshaping/constraint/LoopshapingStateInputConstraint.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/loopshaping/constraint/LoopshapingConstraintEliminatePattern.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/loopshaping/constraint/LoopshapingConstraintOutputPattern.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/loopshaping/dynamics/LoopshapingDynamics.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/loopshaping/dynamics/LoopshapingDynamicsEliminatePattern.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/loopshaping/dynamics/LoopshapingDynamicsOutputPattern.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/loopshaping/dynamics/LoopshapingFilterDynamics.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/loopshaping/initialization/LoopshapingInitializer.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/model_data/ModelData.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/model_data/Metrics.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/model_data/Multiplier.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/misc/LinearAlgebra.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/misc/Log.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/soft_constraint/StateSoftConstraint.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/soft_constraint/StateInputSoftConstraint.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/soft_constraint/StateInputSoftBoxConstraint.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/penalties/MultidimensionalPenalty.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/penalties/Penalties.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/penalties/penalties/RelaxedBarrierPenalty.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/penalties/penalties/SquaredHingePenalty.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/thread_support/ThreadPool.cpp"
)
target_include_directories(ocs2_core PUBLIC "${G1_OCS2_SOURCE_DIR}/ocs2_core/include" "${G1_CORE_DIR}" "${G1_CORE_DIR}/whole_body")
target_link_libraries(ocs2_core PUBLIC ocs2_thirdparty ${Boost_LIBRARIES} $<LINK_ONLY:Boost::log> $<LINK_ONLY:Boost::log_setup> $<LINK_ONLY:Boost::system> Eigen3::Eigen OpenMP::OpenMP_CXX Threads::Threads ${CMAKE_DL_LIBS})
target_compile_options(ocs2_core PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wno-ignored-attributes>)

add_library(ocs2_oc SHARED
  "${G1_OCS2_SOURCE_DIR}/ocs2_oc/src/approximate_model/ChangeOfInputVariables.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_oc/src/approximate_model/LinearQuadraticApproximator.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_oc/src/multiple_shooting/Helpers.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_oc/src/multiple_shooting/Initialization.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_oc/src/multiple_shooting/LagrangianEvaluation.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_oc/src/multiple_shooting/MetricsComputation.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_oc/src/multiple_shooting/PerformanceIndexComputation.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_oc/src/multiple_shooting/ProjectionMultiplierCoefficients.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_oc/src/multiple_shooting/Transcription.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_oc/src/oc_data/LoopshapingPrimalSolution.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_oc/src/oc_data/PerformanceIndex.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_oc/src/oc_data/TimeDiscretization.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_oc/src/oc_problem/OptimalControlProblem.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_oc/src/oc_problem/LoopshapingOptimalControlProblem.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_oc/src/oc_problem/OptimalControlProblemHelperFunction.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_oc/src/oc_problem/OcpSize.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_oc/src/oc_problem/OcpToKkt.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_oc/src/oc_solver/SolverBase.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_oc/src/precondition/Ruzi.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_oc/src/rollout/PerformanceIndicesRollout.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_oc/src/rollout/RolloutBase.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_oc/src/rollout/RootFinder.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_oc/src/rollout/InitializerRollout.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_oc/src/rollout/StateTriggeredRollout.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_oc/src/rollout/TimeTriggeredRollout.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_oc/src/rollout/RolloutSettings.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_oc/src/synchronized_module/ReferenceManager.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_oc/src/synchronized_module/LoopshapingReferenceManager.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_oc/src/synchronized_module/LoopshapingSynchronizedModule.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_oc/src/synchronized_module/SolverObserver.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_oc/src/search_strategy/FilterLinesearch.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_oc/src/trajectory_adjustment/TrajectorySpreading.cpp"
)
target_include_directories(ocs2_oc PUBLIC "${G1_OCS2_SOURCE_DIR}/ocs2_oc/include" "${G1_CORE_DIR}" "${G1_CORE_DIR}/whole_body")
target_link_libraries(ocs2_oc PUBLIC ocs2_core)
target_compile_options(ocs2_oc PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wno-ignored-attributes>)

add_library(ocs2_mpc SHARED
  "${G1_OCS2_SOURCE_DIR}/ocs2_mpc/src/LoopshapingSystemObservation.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_mpc/src/MPC_BASE.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_mpc/src/MPC_Settings.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_mpc/src/SystemObservation.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_mpc/src/MRT_BASE.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_mpc/src/MPC_MRT_Interface.cpp"
)
target_include_directories(ocs2_mpc PUBLIC "${G1_OCS2_SOURCE_DIR}/ocs2_mpc/include" "${G1_CORE_DIR}" "${G1_CORE_DIR}/whole_body")
target_link_libraries(ocs2_mpc PUBLIC ocs2_core ocs2_oc)
target_compile_options(ocs2_mpc PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wno-ignored-attributes>)

add_library(ocs2_qp_solver SHARED
  "${G1_OCS2_SOURCE_DIR}/ocs2_test_tools/ocs2_qp_solver/src/Ocs2QpSolver.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_test_tools/ocs2_qp_solver/src/QpDiscreteTranscription.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_test_tools/ocs2_qp_solver/src/QpSolver.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_test_tools/ocs2_qp_solver/src/QpTrajectories.cpp"
)
target_include_directories(ocs2_qp_solver PUBLIC "${G1_OCS2_SOURCE_DIR}/ocs2_test_tools/ocs2_qp_solver/include" "${G1_CORE_DIR}" "${G1_CORE_DIR}/whole_body")
target_link_libraries(ocs2_qp_solver PUBLIC ocs2_core ocs2_oc)
target_compile_options(ocs2_qp_solver PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wno-ignored-attributes>)

add_library(ocs2_ddp SHARED
  "${G1_OCS2_SOURCE_DIR}/ocs2_ddp/src/riccati_equations/ContinuousTimeRiccatiEquations.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_ddp/src/riccati_equations/DiscreteTimeRiccatiEquations.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_ddp/src/riccati_equations/RiccatiModification.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_ddp/src/search_strategy/LevenbergMarquardtStrategy.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_ddp/src/search_strategy/LineSearchStrategy.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_ddp/src/search_strategy/StrategySettings.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_ddp/src/ContinuousTimeLqr.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_ddp/src/GaussNewtonDDP.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_ddp/src/HessianCorrection.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_ddp/src/ILQR.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_ddp/src/SLQ.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_ddp/src/DDP_Settings.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_ddp/src/DDP_HelperFunctions.cpp"
)
target_include_directories(ocs2_ddp PUBLIC "${G1_OCS2_SOURCE_DIR}/ocs2_ddp/include" "${G1_CORE_DIR}" "${G1_CORE_DIR}/whole_body")
target_link_libraries(ocs2_ddp PUBLIC ocs2_core ocs2_oc ocs2_mpc ocs2_qp_solver)
target_compile_options(ocs2_ddp PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wno-ignored-attributes>)

add_library(ocs2_robotic_tools SHARED
  "${G1_OCS2_SOURCE_DIR}/ocs2_robotic_tools/src/common/RotationTransforms.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_robotic_tools/src/common/LoopshapingRobotInterface.cpp"
)
target_include_directories(ocs2_robotic_tools PUBLIC "${G1_OCS2_SOURCE_DIR}/ocs2_robotic_tools/include" "${G1_CORE_DIR}" "${G1_CORE_DIR}/whole_body")
target_link_libraries(ocs2_robotic_tools PUBLIC ocs2_oc ocs2_core ocs2_thirdparty)
target_compile_options(ocs2_robotic_tools PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wno-ignored-attributes>)

add_library(ocs2_pinocchio_interface SHARED
  "${G1_PINOCCHIO_INTERFACE_COMPAT_SOURCE}"
  "${G1_OCS2_SOURCE_DIR}/ocs2_pinocchio/ocs2_pinocchio_interface/src/PinocchioInterfaceCppAd.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_pinocchio/ocs2_pinocchio_interface/src/PinocchioEndEffectorKinematics.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_pinocchio/ocs2_pinocchio_interface/src/PinocchioEndEffectorKinematicsCppAd.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_pinocchio/ocs2_pinocchio_interface/src/urdf.cpp"
)
target_include_directories(ocs2_pinocchio_interface PUBLIC "${G1_OCS2_SOURCE_DIR}/ocs2_pinocchio/ocs2_pinocchio_interface/include" "${G1_CORE_DIR}" "${G1_CORE_DIR}/whole_body")
target_link_libraries(ocs2_pinocchio_interface PUBLIC ocs2_core ocs2_oc ocs2_robotic_tools pinocchio::pinocchio ${urdfdom_LIBRARIES})
target_compile_options(ocs2_pinocchio_interface PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wno-ignored-attributes>)

add_library(ocs2_centroidal_model SHARED
  "${G1_OCS2_SOURCE_DIR}/ocs2_pinocchio/ocs2_centroidal_model/src/PinocchioCentroidalDynamics.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_pinocchio/ocs2_centroidal_model/src/PinocchioCentroidalDynamicsAD.cpp"
  "${G1_CENTROIDAL_COMPAT_SOURCE}"
  "${G1_OCS2_SOURCE_DIR}/ocs2_pinocchio/ocs2_centroidal_model/src/CentroidalModelPinocchioMapping.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_pinocchio/ocs2_centroidal_model/src/CentroidalModelInfo.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_pinocchio/ocs2_centroidal_model/src/FactoryFunctions.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_pinocchio/ocs2_centroidal_model/src/ModelHelperFunctions.cpp"
)
target_include_directories(ocs2_centroidal_model PUBLIC "${G1_OCS2_SOURCE_DIR}/ocs2_pinocchio/ocs2_centroidal_model/include" "${G1_CORE_DIR}" "${G1_CORE_DIR}/whole_body")
target_link_libraries(ocs2_centroidal_model PUBLIC ocs2_core ocs2_robotic_tools ocs2_pinocchio_interface)
target_compile_options(ocs2_centroidal_model PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wno-ignored-attributes>)

add_library(hpipm_catkin SHARED
  "${G1_OCS2_SOURCE_DIR}/ocs2_sqp/hpipm_catkin/src/HpipmInterface.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_sqp/hpipm_catkin/src/HpipmInterfaceSettings.cpp"
)
target_include_directories(hpipm_catkin PUBLIC "${G1_OCS2_SOURCE_DIR}/ocs2_sqp/hpipm_catkin/include" "${G1_CORE_DIR}" "${G1_CORE_DIR}/whole_body")
target_link_libraries(hpipm_catkin PUBLIC ocs2_core ocs2_oc ocs2_qp_solver hpipm blasfeo)
target_compile_options(hpipm_catkin PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wno-ignored-attributes>)

add_library(ocs2_sqp SHARED
  "${G1_OCS2_SOURCE_DIR}/ocs2_sqp/ocs2_sqp/src/SqpLogging.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_sqp/ocs2_sqp/src/SqpSettings.cpp"
  "${G1_OCS2_SOURCE_DIR}/ocs2_sqp/ocs2_sqp/src/SqpSolver.cpp"
)
target_include_directories(ocs2_sqp PUBLIC "${G1_OCS2_SOURCE_DIR}/ocs2_sqp/ocs2_sqp/include" "${G1_CORE_DIR}" "${G1_CORE_DIR}/whole_body")
target_link_libraries(ocs2_sqp PUBLIC ocs2_core ocs2_mpc ocs2_oc ocs2_qp_solver hpipm_catkin)
target_compile_options(ocs2_sqp PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wno-ignored-attributes>)

add_library(robot_model SHARED
  "${G1_CORE_DIR}/robot/robot_model/src/RobotDescription.cpp"
  "${G1_CORE_DIR}/robot/robot_model/src/RobotState.cpp"
)
target_include_directories(robot_model PUBLIC "${G1_CORE_DIR}/robot/robot_model/include" "${G1_CORE_DIR}" "${G1_CORE_DIR}/whole_body")
target_link_libraries(robot_model PUBLIC robot_core ${urdfdom_LIBRARIES} absl::flat_hash_map)
target_compile_options(robot_model PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wno-ignored-attributes>)

add_library(humanoid_common_mpc SHARED
  "${G1_CORE_DIR}/whole_body/humanoid_common_mpc/src/contact/ContactCenterPoint.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_common_mpc/src/contact/ContactPolygon.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_common_mpc/src/contact/ContactRectangle.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_common_mpc/src/pinocchio_model/pinocchioUtils.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_common_mpc/src/pinocchio_model/createPinocchioModel.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_common_mpc/src/pinocchio_model/DynamicsHelperFunctions.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_common_mpc/src/common/ModelSettings.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_common_mpc/src/constraint/JointLimitsSoftConstraint.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_common_mpc/src/constraint/FootCollisionConstraint.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_common_mpc/src/constraint/ZeroWrenchConstraint.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_common_mpc/src/constraint/EndEffectorKinematicsTwistConstraint.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_common_mpc/src/constraint/FrictionForceConeConstraint.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_common_mpc/src/constraint/ContactMomentXYConstraintCppAd.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_common_mpc/src/constraint/EndEffectorKinematicsLinearVelConstraint.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_common_mpc/src/cost/StateInputQuadraticCost.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_common_mpc/src/cost/EndEffectorKinematicCostHelpers.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_common_mpc/src/cost/EndEffectorKinematicsQuadraticCost.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_common_mpc/src/cost/ExternalTorqueQuadraticCostAD.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_common_mpc/src/command/TargetTrajectoriesCalculatorBase.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_common_mpc/src/gait/Gait.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_common_mpc/src/gait/GaitSchedule.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_common_mpc/src/gait/ModeSequenceTemplate.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_common_mpc/src/gait/GaitScheduleUpdater.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_common_mpc/src/swing_foot_planner/CubicSpline.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_common_mpc/src/swing_foot_planner/SplineCpg.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_common_mpc/src/swing_foot_planner/SwingTrajectoryPlanner.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_common_mpc/src/reference_manager/SwitchedModelReferenceManager.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_common_mpc/src/reference_manager/ProceduralMpcMotionManager.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_common_mpc/src/HumanoidCostConstraintFactory.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_common_mpc/src/HumanoidPreComputation.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_common_mpc/src/initialization/WeightCompInitializer.cpp"
)
target_include_directories(humanoid_common_mpc PUBLIC "${G1_CORE_DIR}/whole_body/humanoid_common_mpc/include" "${G1_CORE_DIR}" "${G1_CORE_DIR}/whole_body")
target_link_libraries(humanoid_common_mpc PUBLIC g1_ocp_extensions ocs2_core ocs2_mpc ocs2_pinocchio_interface pinocchio::pinocchio)
target_compile_options(humanoid_common_mpc PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wno-ignored-attributes>)

add_library(humanoid_wb_mpc SHARED
  "${G1_CORE_DIR}/whole_body/humanoid_wb_mpc/src/dynamics/DynamicsHelperFunctions.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_wb_mpc/src/dynamics/WBAccelDynamicsAD.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_wb_mpc/src/end_effector/PinocchioEndEffectorDynamicsCppAd.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_wb_mpc/src/constraint/EndEffectorDynamicsLinearAccConstraint.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_wb_mpc/src/constraint/EndEffectorDynamicsAccelerationsConstraint.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_wb_mpc/src/constraint/ZeroAccelerationConstraintCppAd.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_wb_mpc/src/constraint/SwingLegVerticalConstraintCppAd.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_wb_mpc/src/constraint/JointMimicDynamicsConstraint.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_wb_mpc/src/WBMpcInterface.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_wb_mpc/src/WBMpcPreComputation.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_wb_mpc/src/cost/EndEffectorDynamicsCostHelpers.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_wb_mpc/src/cost/EndEffectorDynamicsQuadraticCost.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_wb_mpc/src/cost/EndEffectorDynamicsFootCost.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_wb_mpc/src/cost/JointTorqueCostCppAd.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_wb_mpc/src/command/WBMpcTargetTrajectoriesCalculator.cpp"
  "${G1_CORE_DIR}/whole_body/humanoid_wb_mpc/src/mrt/WBMpcMrtJointController.cpp"
)
target_include_directories(humanoid_wb_mpc PUBLIC "${G1_CORE_DIR}/whole_body/humanoid_wb_mpc/include" "${G1_CORE_DIR}" "${G1_CORE_DIR}/whole_body")
target_link_libraries(humanoid_wb_mpc PUBLIC humanoid_common_mpc robot_model ocs2_sqp ocs2_ddp ocs2_centroidal_model)
target_compile_options(humanoid_wb_mpc PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wno-ignored-attributes>)

# Pinned official SqpSolver.cpp uses ofstream without directly including fstream.
# Supply the standard header from the build adapter; no upstream source edit.
target_compile_options(ocs2_sqp PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-include;fstream>)
