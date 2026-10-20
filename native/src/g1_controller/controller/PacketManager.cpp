#include "controller/PacketManager.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace g1_controller {
ReferencePacket PacketManager::referenceForSolve(uint64_t observationTick) const {
  // 返回本轮参考的副本；校验依据是实际求解观测，而非调度截止时间。
  // 当前运行协议每次初始化建立一个 session，编号固定为 1。
  auto packet = current_;
  if (packet.session != 1) {
    throw std::runtime_error("Reference session does not match the controller session");
  }
  if (packet.tick != observationTick) {
    throw std::runtime_error("Reference tick does not match the solver observation time");
  }
  return packet;
}

void PacketManager::validate(const ReferencePacket& p, double t, double end) {
  if (p.session == 0 || p.sequence == 0)
    throw std::runtime_error("G: Reference identity rejected");
  if (initialized_ &&
      (p.session != previous_.session || p.epoch != previous_.epoch ||
       p.sequence != previous_.sequence + 1 || p.tick <= previous_.tick))
    throw std::runtime_error("G: Reference session/order rejected; fresh process per epoch");

  // 目标时间严格递增，节点维度必须匹配本实例的 OCP。
  auto n = p.target.timeTrajectory.size();
  if ((n != 1 && n != 3) || p.target.stateTrajectory.size() != n ||
      p.target.inputTrajectory.size() != n)
    throw std::runtime_error("G: Target dimensions rejected");
  for (size_t k = 0; k < n; ++k) {
    if (!std::isfinite(p.target.timeTrajectory[k]) ||
        (k && p.target.timeTrajectory[k] <= p.target.timeTrajectory[k - 1]) ||
        p.target.stateTrajectory[k].size() != stateDimension_ ||
        p.target.inputTrajectory[k].size() != inputDimension_ ||
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

  // 外部给出的摆腿参数必须与本实例加载的配置一致。
  if (p.swing != swing_)
    throw std::runtime_error("F: Swing parameters changed from the accepted consumer configuration");
}

void PacketManager::markConsumed(const ReferencePacket& packet) {
  previous_ = packet;
  initialized_ = true;
}
}  // namespace g1_controller
