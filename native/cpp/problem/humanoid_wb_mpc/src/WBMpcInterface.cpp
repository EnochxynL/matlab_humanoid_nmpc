/**
 * @file
 * @brief G1 全身 OCP 的基础装配实现，是理解“优化什么、受什么限制”的主要入口。
 * 读取模型和任务配置，注册动力学、代价、软约束与等式约束；不在此实现 SQP。
 * PlannerContext 直接使用此处创建的 C++ OCP，不再安装生成数学替代包装。
 */
#include "extensions/TermRegistry.h"
/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.
Copyright (c) 2024, 1X Technologies. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <iostream>
#include <string>

// Pinocchio forward declarations must be included first
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "humanoid_wb_mpc/WBMpcInterface.h"

#include <ocs2_core/misc/Display.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/Numerics.h>
#include <ocs2_core/penalties/Penalties.h>
#include "extensions/StateInputSoftConstraint.h"
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

#include <humanoid_common_mpc/pinocchio_model/createPinocchioModel.h>
#include "humanoid_common_mpc/HumanoidCostConstraintFactory.h"
#include "humanoid_common_mpc/initialization/WeightCompInitializer.h"

#include "humanoid_wb_mpc/WBMpcPreComputation.h"
#include "humanoid_wb_mpc/constraint/JointMimicDynamicsConstraint.h"
#include "humanoid_wb_mpc/constraint/SwingLegVerticalConstraintCppAd.h"
#include "humanoid_wb_mpc/constraint/ZeroAccelerationConstraintCppAd.h"
#include "humanoid_wb_mpc/cost/EndEffectorDynamicsFootCost.h"
#include "humanoid_wb_mpc/cost/JointTorqueCostCppAd.h"
#include "humanoid_wb_mpc/dynamics/WBAccelDynamicsAD.h"
#include "humanoid_wb_mpc/end_effector/PinocchioEndEffectorDynamicsCppAd.h"

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief 校验配置文件并加载 MPC、SQP、rollout 等独立设置。
 * 从 URDF 创建 Pinocchio 模型及普通/AD 状态输入映射，建立步态和摆腿参考管理器。
 * setupOCP 为真时组装完整问题；initialState_ 是配置初值，不是实时观测。
 */
WBMpcInterface::WBMpcInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile, bool setupOCP)
    : taskFile_(taskFile), urdfFile_(urdfFile), referenceFile_(referenceFile), modelSettings_(taskFile, urdfFile, "wb_mpc_", "true") {
  // check that task file exists
  boost::filesystem::path taskFilePath(taskFile);
  if (boost::filesystem::exists(taskFilePath)) {
    std::cerr << "[WBMpcInterface] Loading task file: " << taskFilePath << std::endl;
  } else {
    throw std::invalid_argument("[WBMpcInterface] Task file not found: " + taskFilePath.string());
  }
  // check that urdf file exists
  boost::filesystem::path urdfFilePath(urdfFile);
  if (boost::filesystem::exists(urdfFilePath)) {
    std::cerr << "[WBMpcInterface] Loading Pinocchio model from: " << urdfFilePath << std::endl;
  } else {
    throw std::invalid_argument("[WBMpcInterface] URDF file not found: " + urdfFilePath.string());
  }
  // check that targetCommand file exists
  boost::filesystem::path referenceFilePath(referenceFile);
  if (boost::filesystem::exists(referenceFilePath)) {
    std::cerr << "[WBMpcInterface] Loading target command settings from: " << referenceFilePath << std::endl;
  } else {
    throw std::invalid_argument("[WBMpcInterface] targetCommand file not found: " + referenceFilePath.string());
  }

  loadData::loadCppDataType(taskFile, "interface.verbose", verbose_);

  // load setting from loading file
  ddpSettings_ = ddp::loadSettings(taskFile, "ddp", verbose_);
  mpcSettings_ = mpc::loadSettings(taskFile, "mpc", verbose_);
  rolloutSettings_ = rollout::loadSettings(taskFile, "rollout", verbose_);
  sqpSettings_ = sqp::loadSettings(taskFile, "multiple_shooting", verbose_);

  // PinocchioInterface
  pinocchioInterfacePtr_.reset(new PinocchioInterface(createCustomPinocchioInterface(taskFile, urdfFile, modelSettings_)));

  // Setup WB State Input Mapping
  mpcRobotModelPtr_.reset(new WBAccelMpcRobotModel<scalar_t>(modelSettings_));
  mpcRobotModelADPtr_.reset(new WBAccelMpcRobotModel<ad_scalar_t>(modelSettings_));

  // Swing trajectory planner
  std::unique_ptr<SwingTrajectoryPlanner> swingTrajectoryPlanner(
      new SwingTrajectoryPlanner(loadSwingTrajectorySettings(taskFile, "swing_trajectory_config", verbose_), N_CONTACTS));

  // Mode schedule manager
  referenceManagerPtr_ =
      std::make_shared<SwitchedModelReferenceManager>(GaitSchedule::loadGaitSchedule(referenceFile, modelSettings_, verbose_),
                                                      std::move(swingTrajectoryPlanner), *pinocchioInterfacePtr_, *mpcRobotModelPtr_);
  referenceManagerPtr_->setArmSwingReferenceActive(true);

  // initial state
  initialState_.setZero(mpcRobotModelPtr_->getStateDim());
  loadData::loadEigenMatrix(taskFile, "initialState", initialState_);

  if (setupOCP) {
    setupOptimalControlProblem();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

/**
 * @brief 装配交给 OCS2 的 OptimalControlProblem。
 * 动力学：全身加速度模型；代价：状态输入跟踪、终端跟踪、双足任务空间跟踪。
 * 关节范围和足间碰撞进入状态软约束，摩擦锥和接触力矩进入状态输入软约束。
 * 逐足注册零 wrench、支撑/摆动足等式及可选 mimic 约束，激活逻辑由具体类/模式管理。
 * 最后创建预计算、rollout 和重力补偿初始化器。
 * 注意 jointTorqueCost 注册语句当前被注释，不能因为存在工厂函数就认定该项已启用。
 */
void WBMpcInterface::setupOptimalControlProblem() {
  HumanoidCostConstraintFactory factory =
      HumanoidCostConstraintFactory(taskFile_, referenceFile_, *referenceManagerPtr_, *pinocchioInterfacePtr_, *mpcRobotModelPtr_,
                                    *mpcRobotModelADPtr_, modelSettings_, verbose_);

  // Optimal control problem
  problemPtr_.reset(new OptimalControlProblem);

  // Dynamics
  std::unique_ptr<SystemDynamicsBase> dynamicsPtr;
  const std::string modelName = "dynamics";
  dynamicsPtr.reset(new WBAccelDynamicsAD(*pinocchioInterfacePtr_, *mpcRobotModelADPtr_, modelName, modelSettings_));

  problemPtr_->dynamicsPtr = std::move(dynamicsPtr);

  // Cost terms
  ::humanoid::ocp_extensions::addTerm(*problemPtr_->costPtr,"stateInputQuadraticCost", factory.getStateInputQuadraticCost());
  // ::humanoid::ocp_extensions::addTerm(*problemPtr_->costPtr,"jointTorqueCost", getJointTorqueCost(taskFile_));
  ::humanoid::ocp_extensions::addTerm(*problemPtr_->finalCostPtr,"terminalCost", factory.getTerminalCost());

  // Constraints
  ::humanoid::ocp_extensions::addTerm(*problemPtr_->stateSoftConstraintPtr,"jointLimits", factory.getJointLimitsConstraint());
  ::humanoid::ocp_extensions::addTerm(*problemPtr_->stateSoftConstraintPtr,"FootCollisionSoftConstraint", factory.getFootCollisionConstraint());
  // Constraint terms

  EndEffectorDynamicsWeights footTrackingCostWeights =
      EndEffectorDynamicsWeights::getWeights(taskFile_, "task_space_foot_cost_weights.", verbose_);

  // check for mimic joints
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile_, pt);
  bool hasMimicJoints = (pt.find("mimicJoints") != pt.not_found());

  for (size_t i = 0; i < N_CONTACTS; i++) {
    const std::string& footName = modelSettings_.contactNames[i];

    std::unique_ptr<EndEffectorDynamics<scalar_t>> eeDynamicsPtr;
    eeDynamicsPtr.reset(new PinocchioEndEffectorDynamicsCppAd(*pinocchioInterfacePtr_, *mpcRobotModelADPtr_, {footName}, footName,
                                                              modelSettings_.modelFolderCppAd, modelSettings_.recompileLibrariesCppAd,
                                                              modelSettings_.verboseCppAd));

    ::humanoid::ocp_extensions::addTerm(*problemPtr_->softConstraintPtr,footName + "_frictionForceCone", factory.getFrictionForceConeConstraint(i));
    ::humanoid::ocp_extensions::addTerm(*problemPtr_->softConstraintPtr,footName + "_contactMomentXY",
                                        factory.getContactMomentXYConstraint(i, footName + "_contact_moment_XY_constraint"));
    ::humanoid::ocp_extensions::addTerm(*problemPtr_->equalityConstraintPtr,footName + "_zeroWrench", factory.getZeroWrenchConstraint(i));
    ::humanoid::ocp_extensions::addTerm(*problemPtr_->equalityConstraintPtr,footName + "_zeroVelocity", getStanceFootConstraint(*eeDynamicsPtr, i));
    ::humanoid::ocp_extensions::addTerm(*problemPtr_->equalityConstraintPtr,footName + "_normalVelocity", getNormalVelocityConstraint(*eeDynamicsPtr, i));

    if (hasMimicJoints) {
      ::humanoid::ocp_extensions::addTerm(*problemPtr_->equalityConstraintPtr,footName + "_kneeJointMimic", getJointMimicConstraint(i));
    }

    std::string footTrackingCostName = footName + "_TaskSpaceTrackingCost";

    ::humanoid::ocp_extensions::addTerm(*problemPtr_->costPtr,footTrackingCostName, std::unique_ptr<StateInputCost>(new EndEffectorDynamicsFootCost(
                                                        *referenceManagerPtr_, footTrackingCostWeights, *pinocchioInterfacePtr_,
                                                        *eeDynamicsPtr, *mpcRobotModelADPtr_, i, footTrackingCostName, modelSettings_)));
  }

  // Pre-computation
  problemPtr_->preComputationPtr.reset(
      new WBMpcPreComputation(*pinocchioInterfacePtr_, *referenceManagerPtr_->getSwingTrajectoryPlanner(), *mpcRobotModelPtr_));

  // Rollout
  rolloutPtr_.reset(new TimeTriggeredRollout(*problemPtr_->dynamicsPtr, rolloutSettings_));

  // Initialization
  initializerPtr_.reset(new WeightCompInitializer(*pinocchioInterfacePtr_, *referenceManagerPtr_, *mpcRobotModelPtr_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

/**
 * @brief 构造支撑足6维稳定化等式的系数，并交给 ZeroAccelerationConstraintCppAd。
 * 约束综合位姿误差、速度和加速度；具体增益从 footConstraintConfig 读取。
 * 即使注册名称为 zeroVelocity，也不能据名称把它理解为单纯速度等于零。
 */
std::unique_ptr<StateInputConstraint> WBMpcInterface::getStanceFootConstraint(const EndEffectorDynamics<scalar_t>& eeDynamics,
                                                                              size_t contactPointIndex) {
  const ModelSettings::FootConstraintConfig& footCfg = modelSettings_.footConstraintConfig;

  EndEffectorDynamicsAccelerationsConstraint::Config config;
  config.b.setZero(6);
  config.Ax.setZero(6, 6);
  config.Av.setIdentity(6, 6);
  config.Aa.setIdentity(6, 6);
  if (!numerics::almost_eq(footCfg.positionErrorGain_z, 0.0)) {
    config.Ax(2, 2) = footCfg.positionErrorGain_z;
  }
  if (!numerics::almost_eq(footCfg.orientationErrorGain, 0.0)) {
    config.Ax.block(3, 3, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * footCfg.orientationErrorGain;
  }
  config.Av.block(0, 0, 2, 2) = Eigen::MatrixXd::Identity(2, 2) * footCfg.linearVelocityErrorGain_xy;
  config.Av(2, 2) = footCfg.linearVelocityErrorGain_z;
  config.Av.block(3, 3, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * footCfg.angularVelocityErrorGain;
  config.Aa.block(0, 0, 2, 2) = Eigen::MatrixXd::Identity(2, 2) * footCfg.linearAccelerationErrorGain_xy;
  config.Aa(2, 2) = footCfg.linearAccelerationErrorGain_z;
  config.Aa.block(3, 3, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * footCfg.angularAccelerationErrorGain;

  return std::unique_ptr<StateInputConstraint>(
      new ZeroAccelerationConstraintCppAd(*referenceManagerPtr_, eeDynamics, contactPointIndex, config));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief 按左右腿读取 mimic 关节关系及位置/速度稳定化增益，创建动力学等式。
 * 仅在任务配置包含 mimicJoints 时由装配循环调用；不支持其他腿索引。
 */
std::unique_ptr<StateInputConstraint> WBMpcInterface::getJointMimicConstraint(size_t mimicIndex) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile_, pt);
  std::string prefix;
  if (mimicIndex == 0) {
    prefix = "mimicJoints.left_knee.";
  } else if (mimicIndex == 1) {
    prefix = "mimicJoints.right_knee.";
  } else {
    throw std::runtime_error("No mimic joint for index: " + std::to_string(mimicIndex));
  }

  std::string parentJointName;
  std::string childJointName;
  scalar_t multiplier;  // q_child = multiplier* q_parent
  scalar_t positionGain;
  scalar_t velocityGain;

  if (verbose_) {
    std::cerr << "\n #### Joint Mimic Kinematic Constraint Config: ";
    std::cerr << "\n #### "
                 "============================================================="
                 "================\n";
  }
  loadData::loadPtreeValue(pt, parentJointName, prefix + "parentJointName", verbose_);
  loadData::loadPtreeValue(pt, childJointName, prefix + "childJointName", verbose_);
  loadData::loadPtreeValue(pt, multiplier, prefix + "multiplier", verbose_);
  loadData::loadPtreeValue(pt, positionGain, prefix + "positionGain", verbose_);
  loadData::loadPtreeValue(pt, velocityGain, prefix + "velocityGain", verbose_);
  if (verbose_) {
    std::cerr << " #### "
                 "============================================================="
                 "================\n";
  }

  JointMimicDynamicsConstraint::Config config(*mpcRobotModelPtr_, parentJointName, childJointName, multiplier, positionGain, velocityGain);

  return std::unique_ptr<StateInputConstraint>(new JointMimicDynamicsConstraint(*mpcRobotModelPtr_, config));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief 创建摆动腿竖直方向约束；实际表达和模式激活见 SwingLegVerticalConstraintCppAd。
 */
std::unique_ptr<StateInputConstraint> WBMpcInterface::getNormalVelocityConstraint(const EndEffectorDynamics<scalar_t>& eeDynamics,
                                                                                  size_t contactPointIndex) {
  return std::unique_ptr<StateInputConstraint>(new SwingLegVerticalConstraintCppAd(*referenceManagerPtr_, eeDynamics, contactPointIndex));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

/**
 * @brief 读取关节力矩权重并构建力矩代价对象。
 * 当前 setupOptimalControlProblem 未启用此代价；此函数存在不代表它参与求解。
 */
std::unique_ptr<StateInputCost> WBMpcInterface::getJointTorqueCost(const std::string& taskFile) {
  vector_t jointTorqueWeights(mpcRobotModelPtr_->getJointDim());
  loadData::loadEigenMatrix(taskFile, "joint_torque_weights", jointTorqueWeights);
  return std::unique_ptr<StateInputCost>(
      new JointTorqueCostCppAd(jointTorqueWeights, *pinocchioInterfacePtr_, *mpcRobotModelADPtr_, "jointTorqueCost", modelSettings_));
}

}  // namespace ocs2::humanoid
