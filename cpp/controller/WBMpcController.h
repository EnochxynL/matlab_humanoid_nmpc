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

#pragma once
#include <condition_variable>
#include <thread>
#include <exception>
#include <functional>
#include <mutex>
#include <limits>
#include "time/ControlTime.h"
namespace task5c {struct Evidence;}
#include <ocs2_mpc/MPC_MRT_Interface.h>

#include <humanoid_wb_mpc/common/WBAccelMpcRobotModel.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include "interfaces/PolicyObserver.h"
#include <robot_model/ControllerBase.h>
#include "humanoid_common_mpc/reference_manager/ProceduralMpcMotionManager.h"
#include "robot_model/RobotDescription.h"

namespace ocs2::humanoid {

class WaveBControlTimeWBMpcController final : public ::robot::model::ControlBase {
 public:
  void setMathematicsEvidence(std::shared_ptr<task5c::Evidence> state,std::shared_ptr<task5c::Evidence> nominal) {stateEvidence_=std::move(state);nominalInputEvidence_=std::move(nominal);}
 private:
  std::shared_ptr<task5c::Evidence> stateEvidence_,nominalInputEvidence_;
 public:
  /**
   * Constructor.
   *
   * @param [in] mpc: The underlying MPC class to be used.
   * @param [in] topicPrefix: The robot's name.
   * @param [in] mpcDesiredFrequency: The max frequency to run the mpc at.
   */
  WaveBControlTimeWBMpcController(const ::robot::model::RobotDescription& robotDescription,
                          const ModelSettings& modelSettings,
                          MPC_BASE& mpc,
                          PinocchioInterface pinocchioInterface,
                          scalar_t mpcDesiredFrequency = -1,
                          std::shared_ptr<DummyObserver> rVizVisualizerPtr = nullptr);

  /**
   * Destructor.
   */
  ~WaveBControlTimeWBMpcController();

  enum class TimingMode { REALTIME_ASYNCHRONOUS, SIMULATION_TIME_DETERMINISTIC };
  void enableDeterministicSimulation();
  using TraceCallback=std::function<void(const char*,double,double,double)>;
  void setTraceCallback(TraceCallback callback) { trace_ = std::move(callback); }
  bool ready() const { return mcpMrtInterface_.initialPolicyReceived(); }

  /**
   * Handles the low level controller loop that updates the mpc observation, reads out the latest policy and sets the joint control action.
   */

  void computeJointControlAction(scalar_t time,
                                 const ::robot::model::RobotState& robotState,
                                 ::robot::model::RobotJointAction& robotJointAction) override;

  void startMpcThread(const ::robot::model::RobotState& initRobotState);

  // In-process planner/ID boundary. Joint action assembly belongs to Simulink.
  void evaluatePlan(const ::robot::model::RobotState& state, vector_t& x, vector_t& u, vector_t& tau, size_t& mode);

 private:
  /**
   * Handles the MPC solver thread.
   */
  void solverWorker();
  void asynchronousSolverWorker();
  void deterministicSolverWorker();
  void waitForRequest(size_t request);
  void deterministicUpdate(const ::robot::model::RobotState& state);
  TimingMode timingMode_ = TimingMode::REALTIME_ASYNCHRONOUS;
  double mpcControlPeriod_;
  double epoch_ = 0, lastObservationTime_ = -std::numeric_limits<double>::infinity();
  size_t nextSlot_ = 1, requested_ = 0, completed_ = 0;
  double requestDeadline_ = 0;
  SystemObservation requestObservation_;
  std::mutex requestMutex_;
  std::condition_variable requestCv_, completionCv_;
  std::exception_ptr workerFailure_;
  TraceCallback trace_;

  /**
   * Method to convert the latest observation msg to a stable desired trajectory (current position, zero velocity and
   * acceleration)
   *
   * @param [in] msg: The observation message.
   */
  TargetTrajectories currentObservationToResetTrajectory(const SystemObservation& currentMpcObservation);

  void updateMpcState(vector_t& mpcState, const ::robot::model::RobotState& robotState);
  void updateMpcObservation(ocs2::SystemObservation& mpcObservation, const ::robot::model::RobotState& robotState);

  MPC_MRT_Interface mcpMrtInterface_;

  PinocchioInterface pinocchioInterface_;
  ocs2::SystemObservation currentMpcObservation_;
  WBAccelMpcRobotModel<scalar_t> mpcRobotModel_;
  std::vector<size_t> mpcJointIndices_;
  std::vector<size_t> otherJointIndices_;

  size_t mpcDeltaTMicroSeconds_;
  bool realtime_;  // True if MPC is to be run as fast as possible

  std::atomic_bool terminateThread_{false};
  std::jthread solver_worker_;

  std::shared_ptr<DummyObserver> visualizerPtr_;
};

}  // namespace ocs2::humanoid
