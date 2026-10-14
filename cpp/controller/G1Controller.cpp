/**
 * @file
 * @brief G1Controller 的运行时实现：连接公共 API、机器人状态和内部 MPC 控制器。
 * 负责资源所有权、参考包缓存、物理 tick、输入安全检查和策略/力矩身份绑定。
 * 不在这里定义 OCP 方程；OCP 组装见 PlannerContext 和 WBMpcInterface。
 */
#include "G1Controller.h"
#include "problem/PlannerContext.h"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include "data/observation/ObservationConversion.h"
#include "data/reference/ReferenceConversion.h"
#include <stdexcept>

namespace g1_controller {
/**
 * @brief PImpl 私有状态。context 先创建、最后释放，确保被引用的模型和 MPC 存活。
 * lastPolicy 保存95元素策略；lastResult 保存含23关节力矩的118元素兼容结果。
 * actionReady 区分“已查询”与“已完成逆动力学”，lastTick 防止跨 tick 混用。
 */
struct G1Controller::Impl {
  std::unique_ptr<PlannerContext> context;
  std::unique_ptr<robot::model::RobotDescription> description;
  std::unique_ptr<robot::model::RobotState> state;
  std::unique_ptr<WaveBControlTimeWBMpcController> controller;
  std::vector<size_t> ids;
  uint64_t lastTick = UINT64_MAX, sequence = 0;
  vector_t lastResult = vector_t::Zero(118);
  vector_t lastPolicy = vector_t::Zero(95);
  bool actionReady = false;
  double lastRawTime = 0;
  std::ofstream sampleLog;
  std::unique_ptr<CanonicalControlTime> canonicalLog;
/**
 * @brief 按依赖逆序关闭；先释放控制器并等待线程，再销毁机器人状态和 OCP 上下文。
 */
  void close() {
    sampleLog.close();
    canonicalLog.reset();
    controller.reset();
    state.reset();
    description.reset();
    context.reset();
    ids.clear();
    lastTick = UINT64_MAX;
    sequence = 0;
    actionReady = false;
  }
/**
 * @brief 创建 OCP 上下文、状态和日志，并建立按名称排序的29关节索引。
 * 这里只配置确定性调度；实际工作线程在首次 query 时启动。
 */
  void init(const std::string &root, const std::string &folder) {
    if (context)
      throw std::runtime_error(
          "Close existing MEX controller before initialization");
    std::filesystem::create_directories(folder);
    Inputs inputs(root, folder);
    context = std::make_unique<PlannerContext>(inputs);
    context->expectedSession = 1;
    canonicalLog = std::make_unique<CanonicalControlTime>(folder);
    sampleLog.open(folder + "/planner_samples.csv");
    sampleLog << std::setprecision(17)
              << "control_tick,raw_time,canonical_time";
    for (int j = 0; j < 118; ++j)
      sampleLog << ",value" << j;
    sampleLog << "\n";
    description = std::make_unique<robot::model::RobotDescription>(inputs.urdf);
    state = std::make_unique<robot::model::RobotState>(*description);
    auto names = description->getJointNames();
    std::sort(names.begin(), names.end());
    ids = description->getJointIndices(names);
    controller = std::make_unique<WaveBControlTimeWBMpcController>(
        *description, context->interface.modelSettings(), context->mpc,
        context->interface.getPinocchioInterface(),
        context->interface.mpcSettings().mpcDesiredFrequency_,
        context->structure);
    context->configure(*controller);
  }
/**
 * @brief 将内部兼容参考向量转换为 OCP 参考包。
 * 忽略未发布及同 tick 重复包；有效目标数只能为1或3，事件数为1至64。
 * 保存最新包，供求解器同步参考管理回调读取；不是立即求解。
 */
  void reference(const double *a) {
    if (!context)
      throw std::runtime_error("Initialize MEX first");
    if (!a[1])
      return;
    if (sequence && context->currentPacket.tick == static_cast<uint64_t>(a[0]))
      return;
    auto p = g1_controller::reference::decodeReference(a, ++sequence);
    context->currentPacket = std::move(p);
  }
/**
 * @brief 校验物理观测，推进一次2ms物理 tick，并更新策略缓存。
 * 同 tick 返回缓存；新 tick 必须连续且上一 tick 已完成逆动力学。
 * 保留原始后端时间，canonicalLog 校验 tick/time 身份；四元数必须近似单位化。
 * 高度与倾角门禁检查真实观测，不是优化预测轨迹。
 * 当前路径固定两足观测接触标志为真，未来接触切换仍由参考步态决定。
 */
  void query(const double *a) {
    if (!controller)
      throw std::runtime_error("Initialize MEX first");
    uint64_t tick = static_cast<uint64_t>(std::llround(a[0] / .002));
    if (a[3] < .35 || a[3] > 1.3)
      throw std::runtime_error("Base height outside physical safety envelope");
    double pitch =
        std::asin(std::clamp(2 * (a[4] * a[6] - a[7] * a[5]), -1., 1.));
    double roll = std::atan2(2 * (a[4] * a[5] + a[6] * a[7]),
                             1 - 2 * (a[5] * a[5] + a[6] * a[6]));
    if (std::abs(pitch) > .7853981633974483 ||
        std::abs(roll) > .7853981633974483)
      throw std::runtime_error("Tilt outside physical safety envelope");
    if (lastTick == tick)
      return;
    if (lastTick != UINT64_MAX && !actionReady)
      throw std::runtime_error(
          "Complete inverse dynamics before advancing policy tick");
    if (lastTick != UINT64_MAX && tick != lastTick + 1)
      throw std::runtime_error("Nonsequential physical tick");
    double t = canonicalLog->accept(tick, a[0], 1, tick);
    observation::fillRobotState(*state, a, ids, t);
    if (lastTick == UINT64_MAX)
      controller->startMpcThread(*state);
    vector_t x, u;
    size_t mode;
    controller->updatePolicy(*state);
    controller->queryPolicy(x, u, mode);
    lastPolicy << x, u, double(mode), t + .005;
    lastTick = tick;
    lastRawTime = a[0];
    actionReady = false;
  }
/**
 * @brief 验证95元素策略与当前缓存逐元素相同，再计算并缓存前馈力矩。
 * 完成后写一行118元素样本；重复调用不重复求解或记录。
 */
  void inverseDynamics(const double *p) {
    if (!controller || lastTick == UINT64_MAX)
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
/**
 * @brief 保留内部428元素参考适配格式，校验数值后更新参考缓存。
 */
void G1Controller::setReference(const Reference &r) {
  const auto a = g1_controller::reference::encodeReference(r);
  requireFinite(a.data(), a.size());
  impl_->reference(a.data());
}
/**
 * @brief 公共查询入口：结构编码、有限性检查、运行时更新、结果解码。
 */
Policy G1Controller::query(const Observation &observation) {
  const auto a = g1_controller::observation::encodeObservation(observation);
  requireFinite(a.data(), a.size());
  impl_->query(a.data());
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
/**
 * @brief 组合接口严格复用拆分接口，不维护第二套求解或动作算法。
 */
ControlResult G1Controller::step(const Observation &observation) {
  Policy policy = query(observation);
  Torque torque = inverseDynamics(policy);
  return {policy, torque};
}
} // namespace g1_controller
