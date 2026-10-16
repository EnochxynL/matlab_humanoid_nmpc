/**
 * @file
 * @brief 将外部参考接入 OCS2 的求解前回调。
 * 不生成目标或步态；安装外部数据后，委托原参考管理器更新 OCP 所需参考。
 */
#include "extensions/MeasuredMode.h"
#pragma once

#include <memory>
#include <array>
#include <algorithm>
#include "controller/data/time/CanonicalControlTime.h"
#include <ocs2_oc/synchronized_module/ReferenceManagerDecorator.h>
#include <humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h>
#include <functional>
#include <fstream>
#include <iomanip>

namespace g1_controller {

/** @brief 一次参考发布：身份、目标轨迹、接触时间表及固定摆腿配置。 */
struct ReferencePacket {
  ReferencePacket() { schedule.clear(); }

  uint64_t tick = 0, session = 1, epoch = 0, sequence = 0;
  ocs2::TargetTrajectories target;
  ocs2::ModeSchedule schedule;
  ocs2::vector4_t filtered = ocs2::vector4_t::Zero();
  // 位于参考窗口之外的结束标记；不是实际步态切换时刻。
  double terminal = 0;
  std::array<double, 8> swing = {.05, -0.0, .08, -.001, .4, -.15, .3, .005};
};

/**
 * @brief 包装既有 SwitchedModelReferenceManager，保持 OCP 消费者持有的对象不变。
 * PlannerContext 通过 setReferenceManager 将本对象注册给 OCS2。
 * 每轮 preSolverRun 从 Provider 取得参考，检查身份与内容，再委托原管理器。
 * 原管理器的地形适配、摆腿样条及其他 OCP 参考计算继续保留。
 */
class ExternalReferenceManager final : public ocs2::ReferenceManagerDecorator,
                                       public g1::MeasuredModeReceiver {
 public:
  /** @brief 将真实观测的接触模式传给既有参考管理器。 */
  void setMeasuredMode(size_t mode) override { ref_->setMeasuredMode(mode); }

  // 输入本轮求解初始时间，返回由外部数据流安装的参考包。
  using Provider = std::function<ReferencePacket(double)>;

  /**
   * @brief 绑定原管理器、参考来源和消费日志；此时不读取参考、不执行求解。
   * 清空循环步态模板，避免 C++ 再次铺设外部已经给出的接触时间表。
   */
  ExternalReferenceManager(
      std::shared_ptr<ocs2::humanoid::SwitchedModelReferenceManager> ref,
      Provider provider, const std::string& folder)
      : ReferenceManagerDecorator(ref),
        ref_(ref),
        provider_(std::move(provider)),
        log_(folder + "/adapter_consumption.csv") {
    ocs2::humanoid::ModeSequenceTemplate empty({0., 1.}, {3});
    empty.modeSequence.clear();
    empty.switchingTimes.clear();
    ref_->getGaitSchedule()->insertModeSequenceTemplate(empty, 0., 0.);

    log_ << std::setprecision(17)
         << "session,epoch,sequence,control_tick,canonical_control_time,target_count,event_count,event_mode_max_error,target_max_error,legacy_filter_calls,legacy_gait_decisions\n";
  }

  /**
   * @brief OCS2 在实际优化开始前调用：取参考 → 校验 → 安装 → 更新 → 核对并记录。
   * t/end 为本轮预测时域端点；x 为本轮真实观测提供的优化初始状态。
   */
  void preSolverRun(double t, double end, const ocs2::vector_t& x) override {
    ReferencePacket p = provider_(t);
    validate(p, t, end);

    // 原 GaitSchedule 的读取过程会移除末尾标记，因此只补上可移除的结束标记。
    // 不在这里决定实际接触模式，也不改变外部给出的事件时间。
    auto padded = p.schedule;
    padded.eventTimes.push_back(p.terminal);
    padded.modeSequence.push_back(3);
    ref_->getGaitSchedule()->updateModeSchedule(padded);
    ref_->setTargetTrajectories(p.target);
    ref_->preSolverRun(t, end, x);

    // 检查原管理器消费后的参考，防止重复生成步态或意外改写目标。
    const auto& s = ref_->getModeSchedule();
    const auto& target = ref_->getTargetTrajectories();
    if (s.eventTimes != p.schedule.eventTimes ||
        s.modeSequence != p.schedule.modeSequence)
      throw std::runtime_error("H: Adapter altered the authoritative raw schedule");

    double error = 0;
    if (target.timeTrajectory != p.target.timeTrajectory)
      throw std::runtime_error("H: Adapter changed target knot times");
    for (size_t k = 0; k < target.stateTrajectory.size(); ++k)
      error = std::max(error, (target.stateTrajectory[k] - p.target.stateTrajectory[k])
                                 .lpNorm<Eigen::Infinity>());
    if (error != 0)
      throw std::runtime_error("H: Adapter changed target values");

    log_ << p.session << ',' << p.epoch << ',' << p.sequence << ',' << p.tick
         << ',' << t << ',' << p.target.timeTrajectory.size() << ','
         << s.eventTimes.size() << ",0," << error << ",0,0\n" << std::flush;
    previous_ = p;
    initialized_ = true;
  }

 private:
  /**
   * @brief 校验求解时刻的身份、发布顺序、目标维度和接触时间表。
   * t/end 相关检查需要实际求解窗口，所以保留在消费端。
   */
  void validate(const ReferencePacket& p, double t, double end) {
    if (p.session == 0 || p.sequence == 0 || CanonicalControlTime::time(p.tick) != t)
      throw std::runtime_error("G: Reference identity rejected");
    if (initialized_ &&
        (p.session != previous_.session || p.epoch != previous_.epoch ||
         p.sequence != previous_.sequence + 1 || p.tick <= previous_.tick))
      throw std::runtime_error("G: Reference session/order rejected; fresh process per epoch");

    // 目标时间严格递增，每个节点必须包含完整的58维状态和35维输入。
    auto n = p.target.timeTrajectory.size();
    if ((n != 1 && n != 3) || p.target.stateTrajectory.size() != n ||
        p.target.inputTrajectory.size() != n)
      throw std::runtime_error("G: Target dimensions rejected");
    for (size_t k = 0; k < n; ++k) {
      if (!std::isfinite(p.target.timeTrajectory[k]) ||
          (k && p.target.timeTrajectory[k] <= p.target.timeTrajectory[k - 1]) ||
          p.target.stateTrajectory[k].size() != 58 ||
          p.target.inputTrajectory[k].size() != 35 ||
          !p.target.stateTrajectory[k].allFinite() ||
          !p.target.inputTrajectory[k].allFinite())
        throw std::runtime_error("G: Nonfinite or malformed target");
    }

    // m个事件对应m+1个模式；事件允许同一时刻，顺序合同保持原样。
    const auto& e = p.schedule.eventTimes;
    const auto& m = p.schedule.modeSequence;
    if (e.empty() || e.size() > 64 || m.size() != e.size() + 1 ||
        !std::is_sorted(e.begin(), e.end()))
      throw std::runtime_error("G: Schedule shape/order rejected");
    for (double v : e)
      if (!std::isfinite(v))
        throw std::runtime_error("G: Nonfinite event");
    for (auto v : m)
      if (v > 3)
        throw std::runtime_error("G: Unknown mode");

    if (!std::isfinite(p.terminal) || p.terminal <= e.back() ||
        p.terminal <= end + (end - t))
      throw std::runtime_error("G: Terminal marker is not outside the reference window");

    // 固定的已验收摆腿配置；禁止通过接入层静默修改控制任务。
    const std::array<double, 8> expected = {.05, -0.0, .08, -.001, .4, -.15, .3, .005};
    if (p.swing != expected)
      throw std::runtime_error("F: Swing parameters changed from the accepted consumer configuration");
  }

  // 原OCP参考管理器仍由成本、约束等对象共享；本适配器不替换这些引用。
  std::shared_ptr<ocs2::humanoid::SwitchedModelReferenceManager> ref_;
  Provider provider_;
  std::ofstream log_;
  ReferencePacket previous_;
  bool initialized_ = false;
};

}  // namespace g1_controller
