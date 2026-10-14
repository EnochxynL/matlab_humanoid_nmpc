/**
 * @file
 * @brief 机器人状态与 OCS2 MPC/MRT 之间的控制调度层。
 * MPC 更新策略，MRT 在物理控制 tick 查询策略；本文件还提供逆动力学入口。
 * 当前 MEX 路径采用确定性仿真时间调度；保留的异步和完整 JointAction 路径不是
 * 当前拆分 query/inverse_dynamics 接口的执行入口。WaveB 名称来自历史时间接口迁移。
 */
#include "extensions/MeasuredMode.h"
/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.

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

#include "controller/WBMpcController.h"
#include "data/observation/ObservationConversion.h"

#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <humanoid_common_mpc/gait/MotionPhaseDefinition.h>
#include <humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h>
#include <humanoid_common_mpc/reference_manager/ProceduralMpcMotionManager.h>
#include "humanoid_wb_mpc/dynamics/DynamicsHelperFunctions.h"

namespace ocs2::humanoid {

/**
 * @brief 绑定 MPC、刚体模型和关节映射，初始化观测缓存；尚不启动求解线程。
 */
WaveBControlTimeWBMpcController::WaveBControlTimeWBMpcController(const ::robot::model::RobotDescription& robotDescription,
                                                 const ModelSettings& modelSettings,
                                                 MPC_BASE& mpc,
                                                 PinocchioInterface pinocchioInterface,
                                                 scalar_t mpcDesiredFrequency,
                                                 std::shared_ptr<DummyObserver> rVizVisualizerPtr)
    : mcpMrtInterface_(mpc),
      pinocchioInterface_(pinocchioInterface),
      mpcRobotModel_(modelSettings),
      mpcDeltaTMicroSeconds_(1000000 / mpcDesiredFrequency),
      realtime_(mpcDesiredFrequency <= 0),
      mpcControlPeriod_(1.0 / mpcDesiredFrequency),
      visualizerPtr_(rVizVisualizerPtr) {
  mpcJointIndices_ = robotDescription.getJointIndices(modelSettings.mpcModelJointNames);
  otherJointIndices_ = robotDescription.getJointIndices(modelSettings.fixedJointNames);
  currentMpcObservation_.state = vector_t::Zero(mpcRobotModel_.getStateDim());
  currentMpcObservation_.input = vector_t::Zero(mpcRobotModel_.getInputDim());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

/**
 * @brief 通知工作线程退出并唤醒所有等待者，join 后才允许依赖对象销毁。
 */
WaveBControlTimeWBMpcController::~WaveBControlTimeWBMpcController() {
  // Signal the solver thread to terminate
  terminateThread_.store(true);
  requestCv_.notify_all();
  completionCv_.notify_all();

  // Wait for the solver thread to finish if it's joinable
  if (solver_worker_.joinable()) {
    solver_worker_.join();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

/**
 * @brief 从首次真实观测启动求解线程；确定性模式等待首个策略安装后返回。
 * 启动 epoch 使用物理控制时间，不由 wall clock 决定。
 */
void WaveBControlTimeWBMpcController::startMpcThread(const ::robot::model::RobotState& initRobotState) {
  SystemObservation observation;
  observation.state = vector_t::Zero(mpcRobotModel_.getStateDim());
  updateMpcObservation(observation, initRobotState);
  startMpcThread(observation);
}

void WaveBControlTimeWBMpcController::startMpcThread(const SystemObservation& observation) {
  currentMpcObservation_ = observation;
  // Set observation to MPC
  setObservation(currentMpcObservation_);
  if (timingMode_ == TimingMode::SIMULATION_TIME_DETERMINISTIC) {
    epoch_ = ControlTime(observation.time).seconds;
    lastObservationTime_ = epoch_;
    requestObservation_ = currentMpcObservation_;
    requested_ = 1; requestDeadline_ = epoch_;
    if (trace_) trace_("observation_arrived", epoch_, epoch_, 0);
    if (trace_) trace_("update_eligible", epoch_, epoch_, 1);
  }
  solver_worker_ = std::jthread(&WaveBControlTimeWBMpcController::solverWorker, this);
  if (timingMode_ == TimingMode::SIMULATION_TIME_DETERMINISTIC) waitForRequest(1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

/**
 * @brief 委托 observation 模块装填 OCS2 观测；调度器不再内联坐标转换公式。
 */
void WaveBControlTimeWBMpcController::updateMpcObservation(SystemObservation& observation, const ::robot::model::RobotState& state) {
  g1_controller::observation::fillMpcObservation(observation, state, mpcRobotModel_, mpcJointIndices_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

/**
 * @brief 保留的完整机器人动作接口：查询策略、逆动力学并填写关节目标和PD增益。
 * 无策略时使用重力补偿；非 MPC 关节使用独立保持动作。
 * 当前 MEX 拆分路径使用下方 evaluatePolicy/inverseDynamics，JointDrive 在 Simulink 中执行。
 */
void WaveBControlTimeWBMpcController::computeJointControlAction(scalar_t time,
                                                        const ::robot::model::RobotState& robotState,
                                                        ::robot::model::RobotJointAction& robotJointAction) {
  if (timingMode_ == TimingMode::SIMULATION_TIME_DETERMINISTIC) deterministicUpdate(robotState);
  updateMpcObservation(currentMpcObservation_, robotState);
  // Set observation to MPC
  setObservation(currentMpcObservation_);

  vector_t mpcPolicyState;
  vector_t mpcPolicyInput;
  size_t mpcPolicyMode;

  if (mcpMrtInterface_.initialPolicyReceived()) {
    // Evaluate policy with feedback if activated in config
    mcpMrtInterface_.evaluatePolicy(currentMpcObservation_.time + 0.005, currentMpcObservation_.state, mpcPolicyState, mpcPolicyInput,
                                    mpcPolicyMode);

    vector_t mpcJointTorques = computeJointTorques<scalar_t>(mpcPolicyState, mpcPolicyInput, pinocchioInterface_, mpcRobotModel_);
    vector_t mpc_q_desired = mpcRobotModel_.getJointAngles(mpcPolicyState);
    vector_t mpc_qd_desired = mpcRobotModel_.getJointVelocities(mpcPolicyState, mpcPolicyInput);

    // std::cout << "mpcJointTorques: " << mpcJointTorques.transpose() << std::endl;

    for (size_t i = 0; i < mpcJointIndices_.size(); i++) {
      size_t index = mpcJointIndices_[i];
      robot::model::JointAction& action = robotJointAction.at(index).value();

      action.q_des = mpc_q_desired[i];
      action.qd_des = mpc_qd_desired[i];
      action.kp = 1200.0;
      action.kd = 10.0;
      action.feed_forward_effort = mpcJointTorques[i];

      // std::cerr << "MPCtorque!: " << mpcJointTorques[i] << std::endl;
    };

    if (visualizerPtr_ != nullptr) {
      visualizerPtr_->update(currentMpcObservation_, mcpMrtInterface_.getPolicy(), mcpMrtInterface_.getCommand());
    }
  }

  else {
    std::cerr << "Apply weight compensating torque..." << std::endl;
    //   Apply weight compensated input around current state
    mpcPolicyState = currentMpcObservation_.state;
    mpcPolicyInput = weightCompensatingInput(pinocchioInterface_, {true, true}, mpcRobotModel_);
    vector_t weightCompensatingTorques = computeJointTorques<scalar_t>(mpcPolicyState, mpcPolicyInput, pinocchioInterface_, mpcRobotModel_);

    for (size_t i = 0; i < mpcJointIndices_.size(); i++) {
      size_t index = mpcJointIndices_[i];
      robot::model::JointAction& action = robotJointAction.at(index).value();

      action.q_des = 0;
      action.qd_des = 0;
      action.kp = 0;
      action.kd = 0;
      action.feed_forward_effort = weightCompensatingTorques[i];
    };
  }

  for (size_t i = 0; i < otherJointIndices_.size(); i++) {
    size_t index = otherJointIndices_[i];
    robot::model::JointAction& action = robotJointAction.at(index).value();

    action.q_des = 0;
    action.qd_des = 0;
    action.kp = 100;
    action.kd = 1.0;
    action.feed_forward_effort = 0.0;
  };
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

/**
 * @brief 保留的异步模式：循环求解并按墙钟节拍等待；当前确定性 MEX 路径不走此分支。
 */
void WaveBControlTimeWBMpcController::asynchronousSolverWorker() {
  mcpMrtInterface_.resetMpcNode(currentObservationToResetTrajectory(latestObservation()));
  std::cerr << "MPC is reset. NMPC solver started!" << std::endl;

  while (!terminateThread_.load()) {
    auto targetTimeForNextIteration = std::chrono::steady_clock::now() + std::chrono::microseconds(mpcDeltaTMicroSeconds_);

    advanceMpc();

    // Publish if Policy has been updated
    if (!mcpMrtInterface_.updatePolicy()) {
      std::cerr << "The solver has failed to update!!" << std::endl;
      return;
    }

    // std::cerr << "MPC policy computed!" << std::endl;

    if (!realtime_) {
      auto currentTime = std::chrono::steady_clock::now();
      if (currentTime > targetTimeForNextIteration) {
        auto delay = std::chrono::duration_cast<std::chrono::microseconds>(currentTime - targetTimeForNextIteration).count();

        std::cerr << "Warning: MPC loop running slow by " << delay << " microseconds." << std::endl;
      } else {
        // Sleep in case sim loop is faster than specified
        std::this_thread::sleep_until(targetTimeForNextIteration);
      }
    }
  }
  std::cerr << "Shutting down NMPC" << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief 构造 MPC 首次重置目标：保留当前位置，清零速度及 pitch/roll。
 * 这是求解器启动目标，不会直接修改真实机器人状态。
 */
TargetTrajectories WaveBControlTimeWBMpcController::currentObservationToResetTrajectory(const SystemObservation& currentObservation) {
  vector_t targetState = currentObservation.state;

  // zero out velocities
  targetState.tail(mpcRobotModel_.getGenCoordinatesDim()) = vector_t::Zero(mpcRobotModel_.getGenCoordinatesDim());

  // zero out pitch + roll angles
  targetState.segment<2>(4) = vector_t::Zero(2);

  const TargetTrajectories resetTargetTrajectories({currentObservation.time}, {targetState},
                                                   {vector_t::Zero(currentObservation.input.size())});

  std::cerr << "Resetting MPC to current state: \n" << targetState << std::endl;
  return resetTargetTrajectories;
}


/**
 * @brief 在线程启动前启用确定性模式；MPC 周期必须为有限正数。
 */
void WaveBControlTimeWBMpcController::enableDeterministicSimulation() {
  if (solver_worker_.joinable() || !(mpcControlPeriod_ > 0) || !std::isfinite(mpcControlPeriod_))
    throw std::runtime_error("Enable offline mode before worker start with positive MPC frequency");
  timingMode_ = TimingMode::SIMULATION_TIME_DETERMINISTIC;
}
/**
 * @brief 工作线程入口，根据调度模式分发到确定性或异步循环。
 */
void WaveBControlTimeWBMpcController::solverWorker() {
  if (timingMode_ == TimingMode::SIMULATION_TIME_DETERMINISTIC) deterministicSolverWorker();
  else asynchronousSolverWorker();
}
/**
 * @brief 等待指定求解请求完成，将线程异常传播回调用线程。
 * 5秒是墙钟安全超时，不是预测时间、控制周期或 OCP 参数。
 */
void WaveBControlTimeWBMpcController::waitForRequest(size_t request) {
  std::unique_lock<std::mutex> lock(requestMutex_);
  if (!completionCv_.wait_for(lock, std::chrono::seconds(5), [&]{return completed_ >= request || workerFailure_ || terminateThread_.load();}))
    throw std::runtime_error("Offline solver wall safety deadline exceeded");
  if (workerFailure_) std::rethrow_exception(workerFailure_);
  if (completed_ < request) throw std::runtime_error("Worker terminated before requested policy");
}
/**
 * @brief 用真实观测时间判断是否到达下一 MPC deadline。
 * 未到期只复用已有策略；到期提交当前观测并等待更新，跳过错过的调度槽。
 * deadline 仅决定求解资格，不把观测时间重标为 deadline。
 */
void WaveBControlTimeWBMpcController::deterministicUpdate(const ::robot::model::RobotState& state) {
  SystemObservation observation;
  observation.state = vector_t::Zero(mpcRobotModel_.getStateDim());
  updateMpcObservation(observation, state);
  deterministicUpdate(observation);
}

void WaveBControlTimeWBMpcController::deterministicUpdate(const SystemObservation& observation) {
  const double t = ControlTime(observation.time).seconds;
  if (t < lastObservationTime_ - 1e-12) throw std::runtime_error("Control time rollback: fresh epoch/process required");
  if (trace_) trace_("observation_arrived", t, epoch_ + nextSlot_ * mpcControlPeriod_, 0);
  lastObservationTime_ = t;
  const double deadline = epoch_ + nextSlot_ * mpcControlPeriod_;
  if (t + 1e-12 < deadline) return;
  size_t skipped = 0;
  while (epoch_ + (nextSlot_ + 1) * mpcControlPeriod_ <= t + 1e-12) { ++nextSlot_; ++skipped; }
  ++nextSlot_;
  currentMpcObservation_ = observation;
  size_t request;
  {
    std::lock_guard<std::mutex> lock(requestMutex_);
    requestObservation_ = currentMpcObservation_; requestDeadline_ = deadline; request = ++requested_;
  }
  if (trace_) trace_("update_eligible", t, deadline, request);
  if (skipped && trace_) trace_("skipped_control_slots", t, deadline, skipped);
  requestCv_.notify_one();
  waitForRequest(request);
}
/**
 * @brief 串行处理求解请求：读取观测 → 首次重置 → advanceMpc → 安装策略 → 通知完成。
 * 计算耗时不推进仿真时间；失败保存 exception_ptr 并唤醒主线程。
 */
void WaveBControlTimeWBMpcController::deterministicSolverWorker() {
  try {
    bool reset = true;
    while (!terminateThread_.load()) {
      SystemObservation observation; size_t request; double deadline;
      {
        std::unique_lock<std::mutex> lock(requestMutex_);
        requestCv_.wait(lock, [&]{return terminateThread_.load() || requested_ > completed_;});
        if (terminateThread_.load()) break;
        observation = requestObservation_; request = requested_; deadline = requestDeadline_;
      }
      setObservation(observation);
      if (reset) { mcpMrtInterface_.resetMpcNode(currentObservationToResetTrajectory(observation)); reset = false; }
      if (trace_) trace_("solver_started", observation.time, deadline, request);
      advanceMpc();
      if (trace_) trace_("solver_finished", observation.time, deadline, request);
      if (!mcpMrtInterface_.updatePolicy()) throw std::runtime_error("Requested offline solve did not install a policy");
      if (trace_) trace_("policy_installed", observation.time, deadline, request);
      {
        std::lock_guard<std::mutex> lock(requestMutex_); completed_ = request;
      }
      completionCv_.notify_one();
    }
  } catch (...) {
    { std::lock_guard<std::mutex> lock(requestMutex_); workerFailure_ = std::current_exception(); }
    completionCv_.notify_all();
  }
}

/**
 * @brief 当前 MEX 的策略入口：必要时更新 MPC，再在观测时间+5ms处调用 MRT。
 * 当前观测状态也传给 MRT；是否使用反馈由其实际控制器及配置决定。
 * 检查策略状态/输入有限性；此函数尚不计算关节力矩。
 */
void WaveBControlTimeWBMpcController::updatePolicy(const ::robot::model::RobotState& state) {
  SystemObservation observation;
  observation.state = vector_t::Zero(mpcRobotModel_.getStateDim());
  updateMpcObservation(observation, state);
  updatePolicy(observation);
}

void WaveBControlTimeWBMpcController::updatePolicy(const SystemObservation& observation) {
  policyQueryPrepared_ = false;
  if (timingMode_ != TimingMode::SIMULATION_TIME_DETERMINISTIC)
    throw std::runtime_error("MEX requires deterministic simulation scheduling");
  deterministicUpdate(observation);
  currentMpcObservation_ = observation;
  setObservation(currentMpcObservation_);
  if (!mcpMrtInterface_.initialPolicyReceived()) throw std::runtime_error("No publishable OCS2 policy");
  policyQueryPrepared_ = true;
}

// 只查询缓存策略；不会触发一次新的 MPC 求解。
void WaveBControlTimeWBMpcController::queryPolicy(vector_t& x, vector_t& u, size_t& mode) {
  if (!policyQueryPrepared_) throw std::runtime_error("Prepare current observation before policy query");
  mcpMrtInterface_.evaluatePolicy(currentMpcObservation_.time + 0.005, currentMpcObservation_.state, x, u, mode);
  if (!x.allFinite() || !u.allFinite()) throw std::runtime_error("Nonfinite policy output");
}

void WaveBControlTimeWBMpcController::evaluatePolicy(const ::robot::model::RobotState& state,
                                                    vector_t& x, vector_t& u, size_t& mode) {
  updatePolicy(state);
  queryPolicy(x, u, mode);
}

/**
 * @brief 用策略状态和35维输入计算23关节前馈力矩，并通知诊断观察器。
 * 输入包含接触 wrench 和关节加速度；PD反馈及执行饱和不在这里计算。
 */
vector_t WaveBControlTimeWBMpcController::inverseDynamics(const vector_t& x, const vector_t& u) {
  if (x.size() != 58 || u.size() != 35 || !x.allFinite() || !u.allFinite())
    throw std::runtime_error("Invalid inverse dynamics inputs");
  vector_t tau = computeJointTorques<scalar_t>(x, u, pinocchioInterface_, mpcRobotModel_);
  if (visualizerPtr_) visualizerPtr_->update(currentMpcObservation_, mcpMrtInterface_.getPolicy(), mcpMrtInterface_.getCommand());
  if (!tau.allFinite()) throw std::runtime_error("Nonfinite inverse dynamics output");
  return tau;
}

/**
 * @brief 保留的组合入口，依次调用策略查询和逆动力学。
 */
void WaveBControlTimeWBMpcController::evaluatePlan(const ::robot::model::RobotState& state,
                                                 vector_t& x, vector_t& u, vector_t& tau, size_t& mode) {
  evaluatePolicy(state, x, u, mode);
  tau = inverseDynamics(x, u);
}

}  // namespace ocs2::humanoid

namespace ocs2::humanoid {
/**
 * @brief 用互斥锁维护应用侧观测副本，同时提交给官方 MRT。
 */
void WaveBControlTimeWBMpcController::setObservation(const SystemObservation& observation) {
 std::lock_guard<std::mutex> lock(observationCopyMutex_);
 observationCopy_ = observation;
 mcpMrtInterface_.setCurrentObservation(observation);
}
/**
 * @brief 返回线程安全的观测副本，避免依赖 fork 曾暴露的内部 getter。
 */
SystemObservation WaveBControlTimeWBMpcController::latestObservation() {
 std::lock_guard<std::mutex> lock(observationCopyMutex_);
 return observationCopy_;
}
/**
 * @brief 将观测接触模式交给 G1 参考扩展，再调用官方 MPC 更新。
 * MeasuredModeReceiver 属于本工程扩展，官方 OCS2 源码无需修改。
 */
void WaveBControlTimeWBMpcController::advanceMpc() {
 auto observation = latestObservation();
 auto* receiver = dynamic_cast<g1::MeasuredModeReceiver*>(&mcpMrtInterface_.getReferenceManager());
 if (!receiver) throw std::runtime_error("G1 reference manager lacks measured-mode receiver");
 receiver->setMeasuredMode(observation.mode);
 mcpMrtInterface_.advanceMpc();
}
}
