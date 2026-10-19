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

#include <functional>
#include <memory>
#include <utility>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <humanoid_wb_mpc/common/WBAccelMpcRobotModel.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

namespace g1_controller {

/** @brief 同步数值后端；调用者决定求解时刻，本类不维护时钟或工作线程。 */
class Ocs2Backend final {
 public:
  Ocs2Backend(const ocs2::humanoid::ModelSettings& settings, ocs2::MPC_BASE& mpc,
              ocs2::PinocchioInterface pinocchio);

  using TraceCallback = std::function<void(const char*, double, double, double)>;
  void setTraceCallback(TraceCallback callback) { trace_ = std::move(callback); }
  using PolicyCallback = std::function<void(const ocs2::SystemObservation&,
      const ocs2::PrimalSolution&, const ocs2::CommandData&)>;
  void setPolicyCallback(PolicyCallback callback) { policy_ = std::move(callback); }
  bool ready() const { return mrt_.initialPolicyReceived(); }
  // request=false 仅登记当前观测，继续使用已有策略；首次调用必须求解。
  void prepare(const ocs2::SystemObservation& observation, bool request);
  void queryPolicy(ocs2::vector_t& state, ocs2::vector_t& input, size_t& mode);
  ocs2::vector_t inverseDynamics(const ocs2::vector_t& state, const ocs2::vector_t& input);

 private:
  ocs2::TargetTrajectories resetTarget(const ocs2::SystemObservation& observation) const;
  ocs2::MPC_MRT_Interface mrt_;
  ocs2::PinocchioInterface pinocchio_;
  ocs2::humanoid::WBAccelMpcRobotModel<ocs2::scalar_t> robotModel_;
  ocs2::SystemObservation observation_;
  PolicyCallback policy_;
  TraceCallback trace_;
  size_t solveCount_ = 0;
  bool prepared_ = false;
  bool failed_ = false;
};
} // namespace g1_controller
