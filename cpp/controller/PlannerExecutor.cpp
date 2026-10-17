/**
 * @file
 * @brief 机器人状态与 OCS2 MPC/MRT 之间的控制调度层。
 * MPC 更新策略，MRT 在物理控制 tick 查询策略；本文件还提供逆动力学入口。
 * 当前路径仅保留确定性仿真时间调度；动作装填由 Simulink 执行。
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

#include "controller/PlannerExecutor.h"

#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <humanoid_common_mpc/gait/MotionPhaseDefinition.h>
#include <humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h>
#include <humanoid_common_mpc/reference_manager/ProceduralMpcMotionManager.h>
#include "humanoid_wb_mpc/dynamics/DynamicsHelperFunctions.h"

namespace g1_controller {

/**
 * @brief 绑定 MPC 和刚体模型，初始化观测缓存；尚不启动求解线程。
 */
PlannerExecutor::PlannerExecutor(const ocs2::humanoid::ModelSettings& modelSettings,
                                                 ocs2::MPC_BASE& mpc,
                                                 ocs2::PinocchioInterface pinocchioInterface,
                                                 ocs2::scalar_t mpcDesiredFrequency,
                                                 std::shared_ptr<ocs2::DummyObserver> rVizVisualizerPtr)
    : mcpMrtInterface_(mpc),
      pinocchioInterface_(pinocchioInterface),
      mpcRobotModel_(modelSettings),
      mpcControlPeriod_(1.0 / mpcDesiredFrequency),
      visualizerPtr_(rVizVisualizerPtr) {
  currentMpcObservation_.state = ocs2::vector_t::Zero(mpcRobotModel_.getStateDim());
  currentMpcObservation_.input = ocs2::vector_t::Zero(mpcRobotModel_.getInputDim());
}

/**
 * @brief 通知工作线程退出并唤醒所有等待者，join 后才允许依赖对象销毁。
 */
PlannerExecutor::~PlannerExecutor() {
  // Signal the solver thread to terminate
  terminateThread_.store(true);
  requestCv_.notify_all();
  completionCv_.notify_all();

  // Wait for the solver thread to finish if it's joinable
  if (solver_worker_.joinable()) {
    solver_worker_.join();
  }
}

void PlannerExecutor::startMpcThread(const ocs2::SystemObservation& observation) {
  if (solver_worker_.joinable())
    throw std::runtime_error("Solver worker already started");
  if (timingMode_ != TimingMode::SIMULATION_TIME_DETERMINISTIC)
    throw std::runtime_error("Enable deterministic scheduling before starting solver");
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
  solver_worker_ = std::thread(&PlannerExecutor::solverWorker, this);
  if (timingMode_ == TimingMode::SIMULATION_TIME_DETERMINISTIC) waitForRequest(1);
}

/**
 * @brief 构造 MPC 首次重置目标：保留当前位置，清零速度及 pitch/roll。
 * 这是求解器启动目标，不会直接修改真实机器人状态。
 */
ocs2::TargetTrajectories PlannerExecutor::currentObservationToResetTrajectory(const ocs2::SystemObservation& currentObservation) {
  ocs2::vector_t targetState = currentObservation.state;

  // zero out velocities
  targetState.tail(mpcRobotModel_.getGenCoordinatesDim()) = ocs2::vector_t::Zero(mpcRobotModel_.getGenCoordinatesDim());

  // zero out pitch + roll angles
  targetState.segment<2>(4) = ocs2::vector_t::Zero(2);

  const ocs2::TargetTrajectories resetTargetTrajectories({currentObservation.time}, {targetState},
                                                   {ocs2::vector_t::Zero(currentObservation.input.size())});

  std::cerr << "Resetting MPC to current state: \n" << targetState << std::endl;
  return resetTargetTrajectories;
}

/**
 * @brief 在线程启动前启用确定性模式；MPC 周期必须为有限正数。
 */
void PlannerExecutor::enableDeterministicSimulation() {
  if (solver_worker_.joinable() || !(mpcControlPeriod_ > 0) || !std::isfinite(mpcControlPeriod_))
    throw std::runtime_error("Enable offline mode before worker start with positive MPC frequency");
  timingMode_ = TimingMode::SIMULATION_TIME_DETERMINISTIC;
}
/**
 * @brief 工作线程入口，只接受显式启用的确定性调度。
 */
void PlannerExecutor::solverWorker() {
  if (timingMode_ == TimingMode::SIMULATION_TIME_DETERMINISTIC) deterministicSolverWorker();
  else throw std::runtime_error("Enable deterministic scheduling before starting solver");
}
/**
 * @brief 等待指定求解请求完成，将线程异常传播回调用线程。
 * 5秒是墙钟安全超时，不是预测时间、控制周期或 OCP 参数。
 */
void PlannerExecutor::waitForRequest(size_t request) {
  std::unique_lock<std::mutex> lock(requestMutex_);
  if (!completionCv_.wait_for(lock, std::chrono::seconds(5), [&]{return completed_ >= request || workerFailure_ || terminateThread_.load();}))
    throw std::runtime_error("Offline solver wall safety deadline exceeded");
  if (workerFailure_) std::rethrow_exception(workerFailure_);
  if (completed_ < request) throw std::runtime_error("Worker terminated before requested policy");
}

void PlannerExecutor::deterministicUpdate(const ocs2::SystemObservation& observation) {
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
void PlannerExecutor::deterministicSolverWorker() {
  try {
    bool reset = true;
    while (!terminateThread_.load()) {
      ocs2::SystemObservation observation; size_t request; double deadline;
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

void PlannerExecutor::updatePolicy(const ocs2::SystemObservation& observation) {
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
void PlannerExecutor::queryPolicy(ocs2::vector_t& x, ocs2::vector_t& u, size_t& mode) {
  if (!policyQueryPrepared_) throw std::runtime_error("Prepare current observation before policy query");
  mcpMrtInterface_.evaluatePolicy(currentMpcObservation_.time + 0.005, currentMpcObservation_.state, x, u, mode);
  if (!x.allFinite() || !u.allFinite()) throw std::runtime_error("Nonfinite policy output");
}

/**
 * @brief 用策略状态和35维输入计算23关节前馈力矩，并通知诊断观察器。
 * 输入包含接触 wrench 和关节加速度；PD反馈及执行饱和不在这里计算。
 */
ocs2::vector_t PlannerExecutor::inverseDynamics(const ocs2::vector_t& x, const ocs2::vector_t& u) {
  if (x.size() != 58 || u.size() != 35 || !x.allFinite() || !u.allFinite())
    throw std::runtime_error("Invalid inverse dynamics inputs");
  ocs2::vector_t tau = ocs2::humanoid::computeJointTorques<ocs2::scalar_t>(x, u, pinocchioInterface_, mpcRobotModel_);
  if (visualizerPtr_) visualizerPtr_->update(currentMpcObservation_, mcpMrtInterface_.getPolicy(), mcpMrtInterface_.getCommand());
  if (!tau.allFinite()) throw std::runtime_error("Nonfinite inverse dynamics output");
  return tau;
}

}  // namespace g1_controller

namespace g1_controller {
/**
 * @brief 用互斥锁维护应用侧观测副本，同时提交给官方 MRT。
 */
void PlannerExecutor::setObservation(const ocs2::SystemObservation& observation) {
 std::lock_guard<std::mutex> lock(observationCopyMutex_);
 observationCopy_ = observation;
 mcpMrtInterface_.setCurrentObservation(observation);
}
/**
 * @brief 返回线程安全的观测副本，避免依赖 fork 曾暴露的内部 getter。
 */
ocs2::SystemObservation PlannerExecutor::latestObservation() {
 std::lock_guard<std::mutex> lock(observationCopyMutex_);
 return observationCopy_;
}
/**
 * @brief 将观测接触模式交给 G1 参考扩展，再调用官方 MPC 更新。
 * MeasuredModeReceiver 属于本工程扩展，官方 OCS2 源码无需修改。
 */
void PlannerExecutor::advanceMpc() {
 auto observation = latestObservation();
 auto* receiver = dynamic_cast<::humanoid::ocp_extensions::MeasuredModeReceiver*>(&mcpMrtInterface_.getReferenceManager());
 if (!receiver) throw std::runtime_error("G1 reference manager lacks measured-mode receiver");
 receiver->setMeasuredMode(observation.mode);
 mcpMrtInterface_.advanceMpc();
}
}
