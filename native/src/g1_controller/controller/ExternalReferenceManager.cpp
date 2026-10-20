#include "controller/ExternalReferenceManager.h"

#include "controller/diagnostics/Diagnostics.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace g1_controller {
ExternalReferenceManager::ExternalReferenceManager(
    std::shared_ptr<ocs2::humanoid::SwitchedModelReferenceManager> originalManager,
    PacketManager& packets, diagnostics::Diagnostics* diagnostics)
    : ReferenceManagerDecorator(originalManager), ref_(std::move(originalManager)),
      packets_(packets), diagnostics_(diagnostics) {
  // 外部已给出接触时间表，清空原管理器用于自动延展的循环模板。
  ocs2::humanoid::ModeSequenceTemplate empty({0., 1.}, {3});
  empty.modeSequence.clear();
  empty.switchingTimes.clear();
  ref_->getGaitSchedule()->insertModeSequenceTemplate(empty, 0., 0.);
}

// OCS2 调用的主入口：取参考、校验、安装、更新原管理器、核对并记录消费。
void ExternalReferenceManager::preSolverRun(double t, double end, const ocs2::vector_t& x) {
  ReferencePacket packet = packets_.referenceForSolve(t);
  packets_.validate(packet, t, end);

  if (diagnostics_) {
    diagnostics_->reference(t, packet.filtered);
  }

  // 原 GaitSchedule 的读取过程会移除末尾标记，因此只补上可移除的结束标记。
  // 不在这里决定实际接触模式，也不改变外部给出的事件时间。
  auto padded = packet.schedule;
  padded.eventTimes.push_back(packet.terminal);
  padded.modeSequence.push_back(3);
  ref_->getGaitSchedule()->updateModeSchedule(padded);
  ref_->setTargetTrajectories(packet.target);

  // 执行
  ref_->preSolverRun(t, end, x);

  // 检查原管理器消费后的参考，防止重复生成步态或意外改写目标。
  const auto& s = ref_->getModeSchedule();
  const auto& target = ref_->getTargetTrajectories();
  if (s.eventTimes != packet.schedule.eventTimes ||
      s.modeSequence != packet.schedule.modeSequence)
    throw std::runtime_error("H: Adapter altered the authoritative raw schedule");

  double error = 0;
  if (target.timeTrajectory != packet.target.timeTrajectory)
    throw std::runtime_error("H: Adapter changed target knot times");
  for (size_t k = 0; k < target.stateTrajectory.size(); ++k)
    error = std::max(error, (target.stateTrajectory[k] - packet.target.stateTrajectory[k])
                               .lpNorm<Eigen::Infinity>());
  if (error != 0)
    throw std::runtime_error("H: Adapter changed target values");

  if (diagnostics_) diagnostics_->consumed(packet, t, error);

  packets_.markConsumed(packet);
}

}  // namespace g1_controller
