/**
 * @file
 * @brief G1Controller 的运行时实现：连接公共 API、已提取观测和内部 MPC 控制器。
 * 负责资源所有权、参考包缓存、物理 tick、输入安全检查和策略/力矩身份绑定。
 * 不在这里定义 OCP 方程；OCP 组装见 PlannerContext 和 WBMpcInterface。
 */
#include "G1Controller.h"
#include "controller/PlannerContext.h"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <stdexcept>

namespace g1_controller {
using ocs2::vector_t;
using ocs2::SystemObservation;
/**
 * @brief PImpl 私有状态。context 先创建、最后释放，确保被引用的模型和 MPC 存活。
 * lastPolicy 保存95元素策略；lastResult 保存含23关节力矩的118元素兼容结果。
 * actionReady 区分“已查询”与“已完成逆动力学”，lastTick 防止跨 tick 混用。
 */
struct G1Controller::Impl {
  std::unique_ptr<PlannerContext> context;
  std::unique_ptr<PlannerExecutor> controller;
  uint64_t lastTick = UINT64_MAX, sequence = 0;
  vector_t lastResult = vector_t::Zero(118);
  vector_t lastPolicy = vector_t::Zero(95);
  bool actionReady = false;
  uint64_t preparedTick = UINT64_MAX;
  bool updateReady = false, expectedUpdate = false;
  SystemObservation preparedObservation;
  double preparedTime = 0;
  double lastRawTime = 0;
  std::ofstream sampleLog;
  std::unique_ptr<CanonicalControlTime> canonicalLog;
/**
 * @brief 按依赖逆序关闭；先释放控制器并等待线程，再销毁 OCP 上下文。
 */
  void close() {
    sampleLog.close();
    canonicalLog.reset();
    controller.reset();
    context.reset();
    lastTick = UINT64_MAX;
    sequence = 0;
    actionReady = false;
    preparedTick = UINT64_MAX;
    updateReady = false;
  }
/**
 * @brief 创建 OCP 上下文和日志；观测转换与关节映射由外部数据流承担。
 * 这里只配置确定性调度；实际工作线程在首次 MPC 更新时启动。
 */
  void init(const std::string &root, const std::string &folder) {
    if (context)
      throw std::runtime_error(
          "Close existing MEX controller before initialization");
    std::filesystem::create_directories(folder);
    PlannerConfig inputs(root, folder);
    context = std::make_unique<PlannerContext>(inputs);
    context->expectedSession = 1;
    canonicalLog = std::make_unique<CanonicalControlTime>(folder);
    sampleLog.open(folder + "/planner_samples.csv");
    sampleLog << std::setprecision(17)
              << "control_tick,raw_time,canonical_time";
    for (int j = 0; j < 118; ++j)
      sampleLog << ",value" << j;
    sampleLog << "\n";
    controller = std::make_unique<PlannerExecutor>(
        context->interface.modelSettings(), context->mpc,
        context->interface.getPinocchioInterface(),
        context->interface.mpcSettings().mpcDesiredFrequency_,
        context->structure);
    context->configure(*controller);
  }
  PlannerTick tickIdentity() const {
    if (!controller || preparedTick == UINT64_MAX)
      throw std::runtime_error("Prepare observation first");
    return {preparedTick, preparedTime, 1.0 / context->interface.mpcSettings().mpcDesiredFrequency_};
  }
  void validateTick(const PlannerTick& token) const {
    const auto current = tickIdentity();
    if (token.tick != current.tick || token.time != current.time || token.mpcPeriod != current.mpcPeriod)
      throw std::runtime_error("Planner tick identity mismatch");
  }
  void update(const PlannerTick& token, bool requested) {
    validateTick(token);
    if (requested != expectedUpdate)
      throw std::runtime_error("External MPC eligibility differs from physical deadline contract");
    if (updateReady) return;
    if (lastTick == UINT64_MAX)
      controller->startMpcThread(preparedObservation);
    controller->updatePolicy(preparedObservation);
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
 * @brief 验证95元素策略与当前缓存逐元素相同，再计算并缓存前馈力矩。
 * 完成后写一行118元素样本；重复调用不重复求解或记录。
 */
  void inverseDynamics(const double *p) {
    if (!controller || lastTick == UINT64_MAX || lastTick != preparedTick)
      throw std::runtime_error("Query a policy before inverse dynamics");
    // Bind the action to exactly the policy queried in this tick; reject
    // stale/mutated packets.
    if (!(Eigen::Map<const vector_t>(p, 95).array() == lastPolicy.array())
             .all())
      throw std::runtime_error("Inverse dynamics policy identity mismatch");
    if (actionReady)
      return;
    vector_t x = lastPolicy.head(58), u = lastPolicy.segment(58, 35);
    vector_t tau = controller->inverseDynamics(x, u);
    lastResult << x, u, tau, lastPolicy[93], lastPolicy[94];
    actionReady = true;
    sampleLog << lastTick << ',' << lastRawTime << ','
              << double(lastTick) * .002;
    for (int j = 0; j < 118; ++j)
      sampleLog << ',' << lastResult[j];
    sampleLog << '\n' << std::flush;
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
void G1Controller::initialize(const ControllerConfig &config) {
  impl_->init(config.projectRoot, config.logDirectory);
}
void G1Controller::shutdown() noexcept { impl_->close(); }
/** @brief 安装具名参考：只构造 OCS2 容器并写入缓存；不再往返428向量。 */
void G1Controller::setReference(const Reference &r) {
  if(!impl_->context) throw std::runtime_error("Initialize MEX first");
  if(!r.publish || (impl_->sequence && impl_->context->currentPacket.tick==r.tick)) return;
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
  impl_->context->currentPacket=std::move(p);
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
  const double t=runtime.canonicalLog->accept(tick,identity[2],1,tick);
  runtime.preparedObservation.state=Eigen::Map<const vector_t>(state.data(),StateDimension);
  runtime.preparedObservation.time=t;
  runtime.preparedObservation.input=vector_t::Zero(35);
  runtime.preparedObservation.mode=3;
  runtime.preparedTime=t;
  runtime.preparedTick=tick;
  runtime.expectedUpdate=runtime.lastTick==UINT64_MAX || runtime.controller->mpcUpdateDue(t);
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
 * @brief 公共力矩入口：检查策略身份并提取缓存中的23维力矩。
 */
Torque G1Controller::inverseDynamics(const Policy &policy) {
  const auto p = encodePolicy(policy);
  requireFinite(p.data(), p.size());
  impl_->inverseDynamics(p.data());
  Torque torque;
  std::copy_n(impl_->lastResult.data() + 93, 23, torque.begin());
  return torque;
}

} // namespace g1_controller
