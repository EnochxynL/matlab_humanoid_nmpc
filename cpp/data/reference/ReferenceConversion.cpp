#include "data/reference/ReferenceConversion.h"
#include <algorithm>
#include <stdexcept>
namespace g1_controller::reference {
std::array<double, 428> encodeReference(const Reference& r) {
  // Keep the accepted reference layout local to the retained OCP adapter.
  std::array<double, 428> a{};
  a[0] = static_cast<double>(r.tick);
  a[1] = r.publish;
  a[2] = static_cast<double>(r.targetCount);
  std::copy(r.targetTimes.begin(), r.targetTimes.end(), a.begin() + 3);
  for (std::size_t k = 0; k < 3; ++k) {
    std::copy(r.targetStates[k].begin(), r.targetStates[k].end(),
              a.begin() + 6 + 58 * k);
    std::copy(r.targetInputs[k].begin(), r.targetInputs[k].end(),
              a.begin() + 180 + 35 * k);
  }
  a[285] = static_cast<double>(r.eventCount);
  std::copy(r.eventTimes.begin(), r.eventTimes.end(), a.begin() + 286);
  for (std::size_t k = 0; k < 65; ++k)
    a[350 + k] = static_cast<double>(r.modes[k]);
  a[415] = r.terminalTime;
  std::copy(r.swingParameters.begin(), r.swingParameters.end(),
            a.begin() + 416);
  std::copy(r.filteredCommand.begin(), r.filteredCommand.end(),
            a.begin() + 424);
  return a;
}
task5a::Packet decodeReference(const double* a, std::uint64_t sequence) {
    task5a::Packet p;
    p.tick = static_cast<uint64_t>(a[0]);
    p.session = 1;
    p.sequence = sequence;
    size_t n = static_cast<size_t>(a[2]), m = static_cast<size_t>(a[285]);
    if ((n != 1 && n != 3) || m < 1 || m > 64)
      throw std::runtime_error("Reference shape rejected");
    for (size_t k = 0; k < n; ++k) {
      p.target.timeTrajectory.push_back(a[3 + k]);
      p.target.stateTrajectory.emplace_back(
          Eigen::Map<const ocs2::vector_t>(a + 6 + 58 * k, 58));
      p.target.inputTrajectory.emplace_back(
          Eigen::Map<const ocs2::vector_t>(a + 180 + 35 * k, 35));
    }
    for (size_t k = 0; k < m; ++k)
      p.schedule.eventTimes.push_back(a[286 + k]);
    for (size_t k = 0; k <= m; ++k)
      p.schedule.modeSequence.push_back(static_cast<size_t>(a[350 + k]));
    p.terminal = a[415];
    for (int k = 0; k < 8; ++k)
      p.swing[k] = a[416 + k];
    for (int k = 0; k < 4; ++k)
      p.filtered[k] = a[424 + k];
  return p;
}
} // namespace g1_controller::reference
