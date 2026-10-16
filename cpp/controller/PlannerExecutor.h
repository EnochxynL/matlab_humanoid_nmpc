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
#include "controller/data/time/ControlTime.h"
#include <ocs2_mpc/MPC_MRT_Interface.h>

#include <humanoid_wb_mpc/common/WBAccelMpcRobotModel.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include "controller/data/diagnostics/PolicyObserver.h"
#include "humanoid_common_mpc/reference_manager/ProceduralMpcMotionManager.h"

namespace g1_controller {

class PlannerExecutor final {
 public:
 private:
  void setObservation(const ocs2::SystemObservation& observation);
  ocs2::SystemObservation latestObservation();
  void advanceMpc();
  std::mutex observationCopyMutex_;
  ocs2::SystemObservation observationCopy_;

 public:
  /**
   * Constructor.
   *
   * @param [in] mpc: The underlying MPC class to be used.
   * @param [in] topicPrefix: The robot's name.
   * @param [in] mpcDesiredFrequency: The max frequency to run the mpc at.
   */
  PlannerExecutor(const ocs2::humanoid::ModelSettings& modelSettings,
                          ocs2::MPC_BASE& mpc,
                          ocs2::PinocchioInterface pinocchioInterface,
                          ocs2::scalar_t mpcDesiredFrequency = -1,
                          std::shared_ptr<ocs2::DummyObserver> rVizVisualizerPtr = nullptr);

  /**
   * Destructor.
   */
  ~PlannerExecutor();

  enum class TimingMode { UNCONFIGURED, SIMULATION_TIME_DETERMINISTIC };
  void enableDeterministicSimulation();
  using TraceCallback=std::function<void(const char*,double,double,double)>;
  void setTraceCallback(TraceCallback callback) { trace_ = std::move(callback); }
  bool ready() const { return mcpMrtInterface_.initialPolicyReceived(); }

  void startMpcThread(const ocs2::SystemObservation& observation);

  // 显式阶段：更新资格判断/求解完成之后，才允许查询已准备观测对应的策略。
  // 仅用于校验外部调度决定；不推进 slot，也不触发求解。
  bool mpcUpdateDue(double time) const { return time + 1e-12 >= epoch_ + nextSlot_ * mpcControlPeriod_; }
  void updatePolicy(const ocs2::SystemObservation& observation);
  void queryPolicy(ocs2::vector_t& x, ocs2::vector_t& u, size_t& mode);
  ocs2::vector_t inverseDynamics(const ocs2::vector_t& x, const ocs2::vector_t& u);

 private:
  /**
   * Handles the MPC solver thread.
   */
  void solverWorker();
  void deterministicSolverWorker();
  void waitForRequest(size_t request);
  void deterministicUpdate(const ocs2::SystemObservation& observation);
  bool policyQueryPrepared_ = false;
  TimingMode timingMode_ = TimingMode::UNCONFIGURED;
  double mpcControlPeriod_;
  double epoch_ = 0, lastObservationTime_ = -std::numeric_limits<double>::infinity();
  size_t nextSlot_ = 1, requested_ = 0, completed_ = 0;
  double requestDeadline_ = 0;
  ocs2::SystemObservation requestObservation_;
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
  ocs2::TargetTrajectories currentObservationToResetTrajectory(const ocs2::SystemObservation& currentMpcObservation);

  ocs2::MPC_MRT_Interface mcpMrtInterface_;

  ocs2::PinocchioInterface pinocchioInterface_;
  ocs2::SystemObservation currentMpcObservation_;
  ocs2::humanoid::WBAccelMpcRobotModel<ocs2::scalar_t> mpcRobotModel_;

  std::atomic_bool terminateThread_{false};
  std::jthread solver_worker_;

  std::shared_ptr<ocs2::DummyObserver> visualizerPtr_;
};

}  // namespace g1_controller
