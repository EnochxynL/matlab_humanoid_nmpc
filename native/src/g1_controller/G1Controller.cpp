/**
 * @file
 * @brief G1Controller 的运行时实现：连接公共 API、已提取观测和内部 MPC 控制器。
 * 负责资源所有权、参考包缓存、物理 tick、输入安全检查和策略/力矩身份绑定。
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
 * lastPolicy 保存95元素策略；lastResult 保存含23关节力矩的118元素兼容结果。
 * actionReady 区分“已查询”与“已完成逆动力学”，lastTick 防止跨 tick 混用。
 */
struct G1Controller::Impl {
  std::unique_ptr<Ocs2Backend> controller;
  uint64_t lastTick = UINT64_MAX, sequence = 0;
  vector_t lastResult = vector_t::Zero(118);
  vector_t lastPolicy = vector_t::Zero(95);
  bool actionReady = false;
  uint64_t preparedTick = UINT64_MAX;
  bool updateReady = false, updateRequested = false;
  SystemObservation preparedObservation;
  double preparedTime = 0;
  double lastRawTime = 0;
  CanonicalControlTime controlTime;
/**
 * @brief 幂等、无异常关闭：后端内部按依赖顺序释放原生对象。
 */
  void close() {
    controlTime = CanonicalControlTime{};
    controller.reset();
    lastTick = UINT64_MAX;
    sequence = 0;
    actionReady = false;
    preparedTick = UINT64_MAX;
    updateReady = false;
  }
/**
 * @brief 创建完整原生后端；外部仍负责观测转换、关节映射和求解调度。
 * 配置由 MATLAB 提供，首次求解由外部更新请求触发。
 */
  void init(const ControllerConfig &config) {
    if (controller)
      throw std::runtime_error(
          "Close existing MEX controller before initialization");
    controller = std::make_unique<Ocs2Backend>(config);
  }
  PlannerTick tickIdentity() const {
    if (!controller || preparedTick == UINT64_MAX)
      throw std::runtime_error("Prepare observation first");
    return {preparedTick, preparedTime, controller->mpcPeriod()};
  }
  void validateTick(const PlannerTick& token) const {
    const auto current = tickIdentity();
    if (token.tick != current.tick || token.time != current.time || token.mpcPeriod != current.mpcPeriod)
      throw std::runtime_error("Planner tick identity mismatch");
  }
  void update(const PlannerTick& token, bool requested) {
    validateTick(token);
    if (updateReady) {
      if (requested != updateRequested)
        throw std::runtime_error("Repeated MPC stage changed its request");
      return;
    }
    controller->prepare(preparedObservation, requested);
    updateRequested = requested;
    updateReady = true;
  }
  void queryPrepared(const PlannerTick& token) {
    validateTick(token);
    if (!updateReady) throw std::runtime_error("Complete MPC update stage before querying policy");
    if (lastTick == preparedTick) return;
    vector_t x, u;
    size_t mode;
    controller->queryPolicy(x, u, mode);
    lastPolicy << x, u, double(mode), preparedTime + .005;
    lastTick = preparedTick;
    actionReady = false;
  }

/**
 * @brief 核对当前策略，接收独立逆动力学计算的力矩并记录完成；本入口不计算力矩。
 * 完成后写一行118元素样本；重复调用不重复求解或记录。
 */
  void completeAction(const double *p, const Torque& torque) {
    if (!controller || lastTick == UINT64_MAX || lastTick != preparedTick)
      throw std::runtime_error("Query a policy before inverse dynamics");
    // Bind the action to exactly the policy queried in this tick; reject
    // stale/mutated packets.
    if (!(Eigen::Map<const vector_t>(p, 95).array() == lastPolicy.array())
             .all())
      throw std::runtime_error("Inverse dynamics policy identity mismatch");
    const Eigen::Map<const vector_t> tau(torque.data(), torque.size());
    if (actionReady) {
      if (!(tau.array() == lastResult.segment(93, 23).array()).all())
        throw std::runtime_error("Repeated action changed its torque");
      return;
    }
    vector_t x = lastPolicy.head(58), u = lastPolicy.segment(58, 35);
    controller->recordPolicyDiagnostics();
    lastResult << x, u, tau, lastPolicy[93], lastPolicy[94];
    actionReady = true;
    if (controller->diagnostics_)
      controller->diagnostics_->sample(lastTick, lastRawTime, preparedTime, lastResult);
  }
};

namespace {
/**
 * @brief 将公共策略转为内部身份校验使用的95元素布局。
 */
std::array<double, 95> encodePolicy(const Policy &p) {
  std::array<double, 95> a{};
  std::copy(p.state.begin(), p.state.end(), a.begin());
  std::copy(p.input.begin(), p.input.end(), a.begin() + 58);
  a[93] = p.mode;
  a[94] = p.queryTime;
  return a;
}
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
  impl_->init(config);
}
void G1Controller::shutdown() noexcept { impl_->close(); }
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
  if(runtime.preparedTick==tick) return runtime.tickIdentity();
  if(runtime.preparedTick!=UINT64_MAX && runtime.preparedTick!=runtime.lastTick)
    throw std::runtime_error("Query prepared observation before advancing tick");
  if(runtime.lastTick!=UINT64_MAX && !runtime.actionReady)
    throw std::runtime_error("Complete inverse dynamics before advancing policy tick");
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
  runtime.lastRawTime=identity[2];
  return runtime.tickIdentity();
}

PlannerTick G1Controller::updateMpc(const PlannerTick& tick, bool requested) {
  impl_->update(tick, requested);
  return tick;
}
Policy G1Controller::queryPrepared(const PlannerTick& tick) {
  impl_->queryPrepared(tick);
  return decodePolicy(impl_->lastPolicy.data());
}
/**
 * @brief 动作完成入口：检查策略与力矩，缓存本 tick 结果并采集诊断。
 */
void G1Controller::completeAction(const Policy &policy, const Torque& torque) {
  const auto p = encodePolicy(policy);
  requireFinite(p.data(), p.size());
  requireFinite(torque.data(), torque.size());
  impl_->completeAction(p.data(), torque);
}

} // namespace g1_controller
