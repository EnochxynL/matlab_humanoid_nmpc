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

#include "controller/Ocs2Backend.h"
#include "extensions/MeasuredMode.h"
#include <humanoid_wb_mpc/dynamics/DynamicsHelperFunctions.h>
#include <stdexcept>

namespace g1_controller {

Ocs2Backend::Ocs2Backend(const ocs2::humanoid::ModelSettings& settings,
                       ocs2::MPC_BASE& mpc, ocs2::PinocchioInterface pinocchio)
    : mrt_(mpc), pinocchio_(std::move(pinocchio)), robotModel_(settings) {}

ocs2::TargetTrajectories Ocs2Backend::resetTarget(const ocs2::SystemObservation& observation) const {
  auto state = observation.state;
  state.tail(robotModel_.getGenCoordinatesDim()).setZero();
  state.segment<2>(4).setZero();
  return {{observation.time}, {state}, {ocs2::vector_t::Zero(observation.input.size())}};
}

void Ocs2Backend::prepare(const ocs2::SystemObservation& observation, bool request) {
  if (failed_) throw std::runtime_error("Previous OCS2 solve failed; close and initialize a fresh backend");
  if (!ready() && !request) throw std::runtime_error("Initial observation requires an MPC solve");
  prepared_ = false;
  observation_ = observation;
  mrt_.setCurrentObservation(observation_);
  if (trace_) trace_("observation_arrived", observation.time, observation.time, 0);
  if (request) {
    // 求解、安装策略均在当前 MEX 调用内完成，异常直接返回调用者。
    // 时间戳是外部请求的实际观测时刻，不再在 C++ 中重算调度截止时间。
    try {
      if (!ready()) mrt_.resetMpcNode(resetTarget(observation));
      auto* receiver = dynamic_cast<::humanoid::ocp_extensions::MeasuredModeReceiver*>(&mrt_.getReferenceManager());
      if (!receiver) throw std::runtime_error("G1 reference manager lacks measured-mode receiver");
      receiver->setMeasuredMode(observation.mode);
      ++solveCount_;
      if (trace_) trace_("update_eligible", observation.time, observation.time, solveCount_);
      if (trace_) trace_("solver_started", observation.time, observation.time, solveCount_);
      mrt_.advanceMpc();
      if (trace_) trace_("solver_finished", observation.time, observation.time, solveCount_);
      if (!mrt_.updatePolicy()) throw std::runtime_error("Requested solve did not install an OCS2 policy");
      if (trace_) trace_("policy_installed", observation.time, observation.time, solveCount_);
    } catch (...) {
      failed_ = true;
      throw;
    }
  }
  if (!ready()) throw std::runtime_error("No publishable OCS2 policy");
  prepared_ = true;
}

void Ocs2Backend::queryPolicy(ocs2::vector_t& state, ocs2::vector_t& input, size_t& mode) {
  if (!prepared_) throw std::runtime_error("Prepare current observation before policy query");
  mrt_.evaluatePolicy(observation_.time + 0.005, observation_.state, state, input, mode);
  if (!state.allFinite() || !input.allFinite()) throw std::runtime_error("Nonfinite policy output");
}

ocs2::vector_t Ocs2Backend::inverseDynamics(const ocs2::vector_t& state, const ocs2::vector_t& input) {
  if (state.size() != 58 || input.size() != 35 || !state.allFinite() || !input.allFinite())
    throw std::runtime_error("Invalid inverse dynamics inputs");
  auto torque = ocs2::humanoid::computeJointTorques<ocs2::scalar_t>(state, input, pinocchio_, robotModel_);
  if (policy_) policy_(observation_, mrt_.getPolicy(), mrt_.getCommand());
  if (!torque.allFinite()) throw std::runtime_error("Nonfinite inverse dynamics output");
  return torque;
}
} // namespace g1_controller
