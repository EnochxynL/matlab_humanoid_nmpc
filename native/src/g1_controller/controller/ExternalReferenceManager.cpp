#include "controller/ExternalReferenceManager.h"

#include "controller/CanonicalControlTime.h"
#include "controller/diagnostics/Diagnostics.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace g1_controller {
ExternalReferenceManager::ExternalReferenceManager(
    std::shared_ptr<ocs2::humanoid::SwitchedModelReferenceManager> originalManager,
    const ReferencePacket& currentPacket, diagnostics::Diagnostics* diagnostics)
    : ReferenceManagerDecorator(originalManager), ref_(std::move(originalManager)),
      currentPacket_(currentPacket), diagnostics_(diagnostics) {
  // 外部已给出接触时间表，清空原管理器用于自动延展的循环模板。
  ocs2::humanoid::ModeSequenceTemplate empty({0., 1.}, {3});
  empty.modeSequence.clear();
  empty.switchingTimes.clear();
  ref_->getGaitSchedule()->insertModeSequenceTemplate(empty, 0., 0.);
}

// OCS2 调用的主入口：取参考、校验、安装、更新原管理器、核对并记录消费。
void ExternalReferenceManager::preSolverRun(double t, double end, const ocs2::vector_t& x) {
  ReferencePacket p = referenceForSolve(t);
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

  if (diagnostics_) diagnostics_->consumed(p, t, error);
  previous_ = p;
  initialized_ = true;
}

ReferencePacket ExternalReferenceManager::referenceForSolve(double observationTime) const {
  // 返回本轮参考的副本；校验依据是实际求解观测，而非调度截止时间。
  // 当前运行协议每次初始化建立一个 session，编号固定为 1。
  auto packet = currentPacket_;
  if (packet.session != 1) {
    throw std::runtime_error("Reference session does not match the controller session");
  }
  if (CanonicalControlTime::time(packet.tick) != observationTime) {
    throw std::runtime_error("Reference tick does not match the solver observation time");
  }
  if (diagnostics_) diagnostics_->reference(observationTime, packet.filtered);
  return packet;
}

void ExternalReferenceManager::validate(const ReferencePacket& p, double t, double end) {
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

}  // namespace g1_controller
