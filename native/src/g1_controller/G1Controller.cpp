/**
 * @file
 * @brief G1Controller 的运行时实现：连接公共 API、已提取观测和内部 MPC 控制器。
 * 负责原生资源、参考缓存、物理 tick 和规划阶段检查；不接收动作力矩。
 * 不在这里定义 OCP 方程；原生对象由 Ocs2Backend 装配，方程由 WBMpcInterface 创建。
 */
#include "g1_controller/G1Controller.h"
#include "controller/Ocs2Backend.h"
#include "controller/diagnostics/Diagnostics.h"
#include <algorithm>
#include <cmath>
#include "controller/CanonicalControlTime.h"
#include <stdexcept>

namespace g1_controller {
using ocs2::vector_t;
using ocs2::SystemObservation;
/**
 * @brief PImpl 私有状态：controller 拥有原生模型、MPC 和诊断对象。
 * lastPolicy 保存当前 tick 的95元素策略；动作计算和结果汇总由外部消费者负责。
 * lastTick 防止跨 tick 混用，查询后允许准备下一个观测。
 */
struct G1Controller::Impl {
  std::unique_ptr<Ocs2Backend> controller;
  uint64_t lastTick = UINT64_MAX, sequence = 0;
  vector_t lastPolicy = vector_t::Zero(95);
  uint64_t preparedTick = UINT64_MAX;
  bool updateReady = false, updateRequested = false;
  SystemObservation preparedObservation;
  double preparedTime = 0;
  CanonicalControlTime controlTime;

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

G1Controller::G1Controller() : impl_(std::make_unique<Impl>()) {}
G1Controller::~G1Controller() { shutdown(); }
DiagnosticBatch G1Controller::drainDiagnostics() {
  if (!impl_->controller) throw std::runtime_error("Initialize MEX first");
  return impl_->controller->diagnostics_ ? impl_->controller->diagnostics_->drain()
                                       : DiagnosticBatch{};
}
void G1Controller::initialize(const ControllerConfig &config) {
  if (impl_->controller)
    throw std::runtime_error("Close existing MEX controller before initialization");
  impl_->controller = std::make_unique<Ocs2Backend>(config);
}

void G1Controller::shutdown() noexcept {
  auto& runtime = *impl_;
  runtime.controlTime = CanonicalControlTime{};
  runtime.controller.reset();
  runtime.lastTick = UINT64_MAX;
  runtime.sequence = 0;
  runtime.preparedTick = UINT64_MAX;
  runtime.updateReady = false;
}

PlannerTick G1Controller::tickIdentity() const {
  const auto& runtime = *impl_;
  if (!runtime.controller || runtime.preparedTick == UINT64_MAX)
    throw std::runtime_error("Prepare observation first");
  return {runtime.preparedTick, runtime.preparedTime, runtime.controller->mpcPeriod()};
}

void G1Controller::validateTick(const PlannerTick& token) const {
  const auto current = tickIdentity();
  if (token.tick != current.tick || token.time != current.time || token.mpcPeriod != current.mpcPeriod)
    throw std::runtime_error("Planner tick identity mismatch");
}
/** @brief 安装具名参考：只构造 OCS2 容器并写入缓存；不再往返428向量。 */
void G1Controller::setReference(const Reference &r) {
  if(!impl_->controller) throw std::runtime_error("Initialize MEX first");
  if(!r.publish || (impl_->sequence && impl_->controller->currentReferenceTick()==r.tick)) return;
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
  impl_->controller->publishReference(std::move(p));
}

PlannerTick G1Controller::installOcpObservation(const std::array<double, StateDimension>& state,
                                               const std::array<double,3>& identity) {
  requireFinite(state.data(),state.size());
  requireFinite(identity.data(),identity.size());
  auto& runtime=*impl_;
  if(!runtime.controller) throw std::runtime_error("Initialize MEX first");
  if(identity[0]<0 || identity[0]>static_cast<double>(uint64_t{1}<<53) || identity[0]!=std::floor(identity[0]))
    throw std::runtime_error("Invalid extracted tick");
  const auto tick=static_cast<uint64_t>(identity[0]);
  if(identity[1]!=CanonicalControlTime::validate(tick,identity[2]))
    throw std::runtime_error("Extracted canonical time identity mismatch");
  if(runtime.preparedTick==tick) return tickIdentity();
  if(runtime.preparedTick!=UINT64_MAX && runtime.preparedTick!=runtime.lastTick)
    throw std::runtime_error("Query prepared observation before advancing tick");
  if(runtime.lastTick!=UINT64_MAX && tick!=runtime.lastTick+1)
    throw std::runtime_error("Nonsequential physical tick");
  const double t=runtime.controlTime.accept(tick,identity[2],1);
  if (runtime.controller->diagnostics_)
    runtime.controller->diagnostics_->time(tick,identity[2],t);
  runtime.preparedObservation.state=Eigen::Map<const vector_t>(state.data(),StateDimension);
  runtime.preparedObservation.time=t;
  runtime.preparedObservation.input=vector_t::Zero(35);
  runtime.preparedObservation.mode=3;
  runtime.preparedTime=t;
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
  runtime.controller->prepare(runtime.preparedObservation, requested);
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
    runtime.controller->queryPolicy(state, input, mode);
    runtime.lastPolicy << state, input, double(mode), runtime.preparedTime + .005;
    runtime.lastTick = runtime.preparedTick;
  }
  return decodePolicy(runtime.lastPolicy.data());
}

} // namespace g1_controller
