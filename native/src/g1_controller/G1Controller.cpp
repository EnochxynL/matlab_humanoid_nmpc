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
/**
 * @file
 * @brief G1Controller 的运行时实现：连接公共 API、已提取观测和内部 MPC 控制器。
 * 负责原生资源、参考缓存、物理 tick 和规划阶段检查；不接收动作力矩。
 * 在这里装配原生对象；OCP 方程仍由 WBMpcInterface 创建。
 */
#include "g1_controller/G1Controller.h"
#include "controller/ExternalReferenceManager.h"
#include <humanoid_wb_mpc/WBMpcInterface.h>
#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include "controller/diagnostics/Diagnostics.h"
#include <algorithm>
#include <cmath>
#include "controller/CanonicalControlTime.h"
#include <stdexcept>

namespace g1_controller {
using ocs2::vector_t;
using ocs2::SystemObservation;
/**
 * @brief PImpl 私有状态：直接持有原生模型、MPC、MRT 和诊断对象。
 * lastPolicy 保存当前 tick 的95元素策略；动作计算和结果汇总由外部消费者负责。
 * lastTick 防止跨 tick 混用，查询后允许准备下一个观测。
 */
struct G1Controller::Impl {
  // 按依赖顺序构造，逆序销毁：MRT/MPC 先于参考包、诊断与模型销毁。
  ocs2::humanoid::WBMpcInterface interface;
  std::unique_ptr<diagnostics::Diagnostics> diagnostics;
  PacketManager packets;
  ocs2::SqpMpc mpc;
  std::shared_ptr<ExternalReferenceManager> referenceManager;
  ocs2::MPC_MRT_Interface mrt;
  bool failed = false;

  explicit Impl(const ControllerConfig& config)
      : interface(config.taskFile, config.urdfFile, config.referenceFile),
        mpc(interface.mpcSettings(), interface.sqpSettings(),
            interface.getOptimalControlProblem(), interface.getInitializer()),
        mrt(mpc) {}
  uint64_t lastTick = UINT64_MAX, sequence = 0;
  vector_t lastPolicy = vector_t::Zero(95);
  uint64_t preparedTick = UINT64_MAX;
  bool updateReady = false, updateRequested = false;
  SystemObservation preparedObservation;

};

namespace {
/**
 * @brief 将内部缓存转回公共策略结构，避免向外暴露 Eigen。
 */
Policy decodePolicy(const double *a) {
  Policy p;
  std::copy_n(a, 58, p.state.begin());
  std::copy_n(a + 58, 35, p.input.begin());
  p.mode = a[93];
  p.queryTime = a[94];
  return p;
}
/**
 * @brief 公共 API 的有限性检查；非法数值不得进入求解器或逆动力学。
 */
void requireFinite(const double *data, std::size_t count) {
  for (std::size_t i = 0; i < count; ++i)
    if (!std::isfinite(data[i]))
      throw std::runtime_error("Nonfinite argument");
}
} // namespace

G1Controller::G1Controller() = default;
G1Controller::~G1Controller() { shutdown(); }
DiagnosticBatch G1Controller::drainDiagnostics() {
  if (!impl_) throw std::runtime_error("Initialize MEX first");
  return impl_->diagnostics ? impl_->diagnostics->drain()
                                       : DiagnosticBatch{};
}
void G1Controller::initialize(const ControllerConfig &config) {
  if (impl_)
    throw std::runtime_error("Close existing MEX controller before initialization");
  auto runtime = std::make_unique<Impl>(config);
  if (config.diagnosticsEnabled)
    runtime->diagnostics = std::make_unique<diagnostics::Diagnostics>(config, runtime->interface, runtime->mpc);
  runtime->referenceManager = std::make_shared<ExternalReferenceManager>(
      runtime->interface.getSwitchedModelReferenceManagerPtr(), runtime->packets, runtime->diagnostics.get());
  runtime->mpc.getSolverPtr()->setReferenceManager(runtime->referenceManager);
  impl_ = std::move(runtime);
}

void G1Controller::shutdown() noexcept {
  impl_.reset();
}

PlannerTick G1Controller::tickIdentity() const {
  if (!impl_ || impl_->preparedTick == UINT64_MAX)
    throw std::runtime_error("Prepare observation first");
  const auto& runtime = *impl_;
  return {runtime.preparedTick, runtime.preparedObservation.time,
          1.0 / runtime.interface.mpcSettings().mpcDesiredFrequency_};
}

void G1Controller::validateTick(const PlannerTick& token) const {
  const auto current = tickIdentity();
  if (token.tick != current.tick || token.time != current.time || token.mpcPeriod != current.mpcPeriod)
    throw std::runtime_error("Planner tick identity mismatch");
}
/** @brief 安装具名参考：只构造 OCS2 容器并写入缓存；不再往返428向量。 */
void G1Controller::setReference(const Reference &r) {
  if(!impl_) throw std::runtime_error("Initialize MEX first");
  if(!r.publish || (impl_->sequence && impl_->packets.currentTick()==r.tick)) return;
  if((r.targetCount!=1 && r.targetCount!=3)||r.eventCount<1||r.eventCount>64)
    throw std::runtime_error("Reference shape rejected");
  ReferencePacket p;
  p.tick=r.tick;p.session=1;p.sequence=++impl_->sequence;
  for(size_t k=0;k<r.targetCount;++k){
    p.target.timeTrajectory.push_back(r.targetTimes[k]);
    p.target.stateTrajectory.emplace_back(Eigen::Map<const vector_t>(r.targetStates[k].data(),58));
    p.target.inputTrajectory.emplace_back(Eigen::Map<const vector_t>(r.targetInputs[k].data(),35));
  }
  p.schedule.eventTimes.assign(r.eventTimes.begin(),r.eventTimes.begin()+r.eventCount);
  p.schedule.modeSequence.assign(r.modes.begin(),r.modes.begin()+r.eventCount+1);
  p.terminal=r.terminalTime;p.swing=r.swingParameters;
  for(size_t k=0;k<4;++k)p.filtered[k]=r.filteredCommand[k];
  impl_->packets.publish(std::move(p));
}

PlannerTick G1Controller::installOcpObservation(const std::array<double, StateDimension>& state,
                                               const std::array<double,3>& identity) {
  requireFinite(state.data(),state.size());
  requireFinite(identity.data(),identity.size());
  if(!impl_) throw std::runtime_error("Initialize MEX first");
  auto& runtime=*impl_;
  if(identity[0]<0 || identity[0]>static_cast<double>(uint64_t{1}<<53) || identity[0]!=std::floor(identity[0]))
    throw std::runtime_error("Invalid extracted tick");
  const auto tick=static_cast<uint64_t>(identity[0]);
  const double t = control_time::validateRawTime(tick, identity[2]);
  if(identity[1]!=t)
    throw std::runtime_error("Extracted canonical time identity mismatch");
  if(runtime.preparedTick==tick) return tickIdentity();
  if(runtime.preparedTick!=UINT64_MAX && runtime.preparedTick!=runtime.lastTick)
    throw std::runtime_error("Query prepared observation before advancing tick");
  // 新会话从 tick 0 开始；其后必须先查询上一 tick，再顺序推进。
  if(runtime.lastTick==UINT64_MAX && tick!=0)
    throw std::runtime_error("First physical tick must be zero");
  if(runtime.lastTick!=UINT64_MAX && tick!=runtime.lastTick+1)
    throw std::runtime_error("Nonsequential physical tick");
  if (runtime.diagnostics)
    runtime.diagnostics->time(tick,identity[2],t);
  runtime.preparedObservation.state=Eigen::Map<const vector_t>(state.data(),StateDimension);
  runtime.preparedObservation.time=t;
  runtime.preparedObservation.input=vector_t::Zero(35);
  runtime.preparedObservation.mode=3;
  runtime.preparedTick=tick;
  runtime.updateReady=false;
  return tickIdentity();
}

PlannerTick G1Controller::updateMpc(const PlannerTick& tick, bool requested) {
  validateTick(tick);
  auto& runtime = *impl_;
  if (runtime.updateReady) {
    if (requested != runtime.updateRequested)
      throw std::runtime_error("Repeated MPC stage changed its request");
    return tick;
  }
  if (runtime.failed)
    throw std::runtime_error("Previous OCS2 solve failed; close and initialize a fresh backend");
  if (!runtime.mrt.initialPolicyReceived() && !requested)
    throw std::runtime_error("Initial observation requires an MPC solve");
  const auto& observation = runtime.preparedObservation;
  runtime.mrt.setCurrentObservation(observation);
  if (runtime.diagnostics) runtime.diagnostics->trace("observation_arrived", observation.time);
  if (requested) {
    try {
      if (!runtime.mrt.initialPolicyReceived()) {
        auto state = observation.state;
        state.tail(runtime.interface.getMpcRobotModel().getGenCoordinatesDim()).setZero();
        state.segment<2>(4).setZero();
        runtime.mrt.resetMpcNode({{observation.time}, {state}, {vector_t::Zero(observation.input.size())}});
      }
      runtime.referenceManager->setMeasuredMode(observation.mode);
      if (runtime.diagnostics) runtime.diagnostics->trace("solver_started", observation.time);
      runtime.mrt.advanceMpc();
      if (runtime.diagnostics) runtime.diagnostics->trace("solver_finished", observation.time);
      if (!runtime.mrt.updatePolicy())
        throw std::runtime_error("Requested solve did not install an OCS2 policy");
      if (runtime.diagnostics) runtime.diagnostics->trace("policy_installed", observation.time);
    } catch (...) {
      runtime.failed = true;
      throw;
    }
  }
  if (!runtime.mrt.initialPolicyReceived()) throw std::runtime_error("No publishable OCS2 policy");
  runtime.updateRequested = requested;
  runtime.updateReady = true;
  return tick;
}

Policy G1Controller::queryPrepared(const PlannerTick& tick) {
  validateTick(tick);
  auto& runtime = *impl_;
  if (!runtime.updateReady)
    throw std::runtime_error("Complete MPC update stage before querying policy");
  if (runtime.lastTick != runtime.preparedTick) {
    vector_t state, input;
    size_t mode;
    const auto& observation = runtime.preparedObservation;
    runtime.mrt.evaluatePolicy(observation.time + .005, observation.state, state, input, mode);
    if (!state.allFinite() || !input.allFinite()) throw std::runtime_error("Nonfinite policy output");
    if (runtime.diagnostics)
      runtime.diagnostics->policy(observation, runtime.mrt.getPolicy(), runtime.mrt.getCommand());
    runtime.lastPolicy << state, input, double(mode), observation.time + .005;
    runtime.lastTick = runtime.preparedTick;
  }
  return decodePolicy(runtime.lastPolicy.data());
}

} // namespace g1_controller
