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

#include <memory>
#include <utility>
#include "controller/ExternalReferenceManager.h"
#include "g1_controller/G1Controller.h"
#include <humanoid_wb_mpc/WBMpcInterface.h>
#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <humanoid_wb_mpc/common/WBAccelMpcRobotModel.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

namespace g1_controller {
namespace diagnostics { class Diagnostics; }

/** @brief 同步数值后端；调用者决定求解时刻，本类不维护时钟或工作线程。 */
class Ocs2Backend final {
 public:
  explicit Ocs2Backend(const ControllerConfig& config);
  ~Ocs2Backend();
  Ocs2Backend(const Ocs2Backend&) = delete;
  Ocs2Backend& operator=(const Ocs2Backend&) = delete;

  double mpcPeriod() const { return 1.0 / interface_.mpcSettings().mpcDesiredFrequency_; }
  uint64_t currentReferenceTick() const { return packets_.currentTick(); }
  void publishReference(ReferencePacket packet) { packets_.publish(std::move(packet)); }
  bool ready() const { return mrt_.initialPolicyReceived(); }
  // request=false 仅登记当前观测，继续使用已有策略；首次调用必须求解。
  void prepare(const ocs2::SystemObservation& observation, bool request);
  void queryPolicy(ocs2::vector_t& state, ocs2::vector_t& input, size_t& mode);

 private:
  ocs2::TargetTrajectories resetTarget(const ocs2::SystemObservation& observation) const;
  // 按依赖顺序构造、逆序销毁：MRT 和 MPC 必须先于参考包、诊断和 OCP 接口销毁。
  ocs2::humanoid::WBMpcInterface interface_;
 public:
  // 可选诊断供控制入口直接记录和提取；初始化后保持此对象存活，供参考管理器使用。
  std::unique_ptr<diagnostics::Diagnostics> diagnostics_;
 private:
  PacketManager packets_;
  ocs2::SqpMpc mpc_;
  std::shared_ptr<ExternalReferenceManager> referenceManager_;
  ocs2::MPC_MRT_Interface mrt_;
  ocs2::SystemObservation observation_;
  bool prepared_ = false;
  bool failed_ = false;
};
} // namespace g1_controller
