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
#include "math/GeneratedNominals.h"
#include "prepareState.h"

#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <humanoid_common_mpc/gait/MotionPhaseDefinition.h>
#include <humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h>
#include <humanoid_common_mpc/reference_manager/ProceduralMpcMotionManager.h>
#include "humanoid_wb_mpc/dynamics/DynamicsHelperFunctions.h"

namespace ocs2::humanoid {

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

void WaveBControlTimeWBMpcController::startMpcThread(const ::robot::model::RobotState& initRobotState) {
  updateMpcObservation(currentMpcObservation_, initRobotState);
  // Set observation to MPC
  setObservation(currentMpcObservation_);
  if (timingMode_ == TimingMode::SIMULATION_TIME_DETERMINISTIC) {
    epoch_ = ControlTime(initRobotState.getTime()).seconds;
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

void WaveBControlTimeWBMpcController::updateMpcState(vector_t& mpcState, const ::robot::model::RobotState& robotState) {
  auto legacy=[&](){
  mpcRobotModel_.setBasePosition(mpcState, robotState.getRootPositionInWorldFrame());
  mpcRobotModel_.setBaseOrientationEulerZYX(mpcState, quaternionToEulerZYX(robotState.getRootRotationLocalToWorldFrame()));

  mpcRobotModel_.setJointAngles(mpcState, robotState.getJointPositions(mpcJointIndices_));

  // currently we send local angular and linear velocity
  mpcRobotModel_.setBaseLinearVelocity(mpcState,
                                       robotState.getRootRotationLocalToWorldFrame() * robotState.getRootLinearVelocityInLocalFrame());
  mpcRobotModel_.setBaseOrientationEulerZYXDerivatives(
      mpcState, getEulerAnglesZyxDerivativesFromLocalAngularVelocity<scalar_t>(mpcRobotModel_.getBaseOrientationEulerZYX(mpcState),
                                                                               robotState.getRootAngularVelocityInLocalFrame()));

  vector_t dummyInput = vector_t::Zero(mpcRobotModel_.getInputDim());
  mpcRobotModel_.setJointVelocities(mpcState, dummyInput, robotState.getJointVelocities(mpcJointIndices_));
  };
  if(!std::getenv("TASK5C_STATE_PREP") || task5c::authorityFor("TASK5C_STATE_AUTHORITY")==task5c::Authority::Legacy){
    if(stateEvidence_)++stateEvidence_->legacyEvaluations;legacy();return;
  }
  auto position=robotState.getRootPositionInWorldFrame();auto rotation=robotState.getRootRotationLocalToWorldFrame();
  double quaternion[4]={rotation.w(),rotation.x(),rotation.y(),rotation.z()};
  auto q=robotState.getJointPositions(mpcJointIndices_),qd=robotState.getJointVelocities(mpcJointIndices_);
  auto linear=robotState.getRootLinearVelocityInLocalFrame(),angular=robotState.getRootAngularVelocityInLocalFrame();
  vector_t generated(58);prepareState(position.data(),quaternion,q.data(),linear.data(),angular.data(),qd.data(),generated.data());
  ++stateEvidence_->value;++stateEvidence_->generated;stateEvidence_->stack();
  if(task5c::authorityFor("TASK5C_STATE_AUTHORITY")==task5c::Authority::Shadow){
    legacy();++stateEvidence_->legacyEvaluations;++stateEvidence_->shadowComparisons;
    if(!(generated.array()==mpcState.array()).all())throw std::runtime_error("State preparation exact shadow mismatch");
  }else mpcState=generated;

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void WaveBControlTimeWBMpcController::updateMpcObservation(ocs2::SystemObservation& mpcObservation, const ::robot::model::RobotState& robotState) {
  updateMpcState(mpcObservation.state, robotState);
  mpcObservation.time = robotState.getTime();
  mpcObservation.input = vector_t::Zero(mpcRobotModel_.getInputDim());  // Add contact forces later.
  std::vector<bool> configContacts = robotState.getContactFlags();
  assert(configContacts.size() == 2);
  contact_flag_t contactFlags;
  std::copy(configContacts.begin(), configContacts.end(), contactFlags.begin());
  mpcObservation.mode = stanceLeg2ModeNumber(contactFlags);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

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
    mpcPolicyInput = task5c::generatedNominalInput(pinocchioInterface_, {true, true}, mpcRobotModel_,nominalInputEvidence_);
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


void WaveBControlTimeWBMpcController::enableDeterministicSimulation() {
  if (solver_worker_.joinable() || !(mpcControlPeriod_ > 0) || !std::isfinite(mpcControlPeriod_))
    throw std::runtime_error("Enable offline mode before worker start with positive MPC frequency");
  timingMode_ = TimingMode::SIMULATION_TIME_DETERMINISTIC;
}
void WaveBControlTimeWBMpcController::solverWorker() {
  if (timingMode_ == TimingMode::SIMULATION_TIME_DETERMINISTIC) deterministicSolverWorker();
  else asynchronousSolverWorker();
}
void WaveBControlTimeWBMpcController::waitForRequest(size_t request) {
  std::unique_lock<std::mutex> lock(requestMutex_);
  if (!completionCv_.wait_for(lock, std::chrono::seconds(5), [&]{return completed_ >= request || workerFailure_ || terminateThread_.load();}))
    throw std::runtime_error("Offline solver wall safety deadline exceeded");
  if (workerFailure_) std::rethrow_exception(workerFailure_);
  if (completed_ < request) throw std::runtime_error("Worker terminated before requested policy");
}
void WaveBControlTimeWBMpcController::deterministicUpdate(const ::robot::model::RobotState& state) {
  const double t = ControlTime(state.getTime()).seconds;
  if (t < lastObservationTime_ - 1e-12) throw std::runtime_error("Control time rollback: fresh epoch/process required");
  if (trace_) trace_("observation_arrived", t, epoch_ + nextSlot_ * mpcControlPeriod_, 0);
  lastObservationTime_ = t;
  const double deadline = epoch_ + nextSlot_ * mpcControlPeriod_;
  if (t + 1e-12 < deadline) return;
  size_t skipped = 0;
  while (epoch_ + (nextSlot_ + 1) * mpcControlPeriod_ <= t + 1e-12) { ++nextSlot_; ++skipped; }
  ++nextSlot_;
  updateMpcObservation(currentMpcObservation_, state);
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

void WaveBControlTimeWBMpcController::evaluatePolicy(const ::robot::model::RobotState& state,
                                                    vector_t& x, vector_t& u, size_t& mode) {
  if (timingMode_ != TimingMode::SIMULATION_TIME_DETERMINISTIC)
    throw std::runtime_error("MEX requires deterministic simulation scheduling");
  deterministicUpdate(state);
  updateMpcObservation(currentMpcObservation_, state);
  setObservation(currentMpcObservation_);
  if (!mcpMrtInterface_.initialPolicyReceived()) throw std::runtime_error("No publishable OCS2 policy");
  mcpMrtInterface_.evaluatePolicy(currentMpcObservation_.time + 0.005, currentMpcObservation_.state, x, u, mode);
  if (!x.allFinite() || !u.allFinite()) throw std::runtime_error("Nonfinite policy output");
}

vector_t WaveBControlTimeWBMpcController::inverseDynamics(const vector_t& x, const vector_t& u) {
  if (x.size() != 58 || u.size() != 35 || !x.allFinite() || !u.allFinite())
    throw std::runtime_error("Invalid inverse dynamics inputs");
  vector_t tau = computeJointTorques<scalar_t>(x, u, pinocchioInterface_, mpcRobotModel_);
  if (visualizerPtr_) visualizerPtr_->update(currentMpcObservation_, mcpMrtInterface_.getPolicy(), mcpMrtInterface_.getCommand());
  if (!tau.allFinite()) throw std::runtime_error("Nonfinite inverse dynamics output");
  return tau;
}

void WaveBControlTimeWBMpcController::evaluatePlan(const ::robot::model::RobotState& state,
                                                 vector_t& x, vector_t& u, vector_t& tau, size_t& mode) {
  evaluatePolicy(state, x, u, mode);
  tau = inverseDynamics(x, u);
}

}  // namespace ocs2::humanoid

namespace ocs2::humanoid {
void WaveBControlTimeWBMpcController::setObservation(const SystemObservation& observation) {
 std::lock_guard<std::mutex> lock(observationCopyMutex_);
 observationCopy_ = observation;
 mcpMrtInterface_.setCurrentObservation(observation);
}
SystemObservation WaveBControlTimeWBMpcController::latestObservation() {
 std::lock_guard<std::mutex> lock(observationCopyMutex_);
 return observationCopy_;
}
void WaveBControlTimeWBMpcController::advanceMpc() {
 auto observation = latestObservation();
 auto* receiver = dynamic_cast<g1::MeasuredModeReceiver*>(&mcpMrtInterface_.getReferenceManager());
 if (!receiver) throw std::runtime_error("G1 reference manager lacks measured-mode receiver");
 receiver->setMeasuredMode(observation.mode);
 mcpMrtInterface_.advanceMpc();
}
}
