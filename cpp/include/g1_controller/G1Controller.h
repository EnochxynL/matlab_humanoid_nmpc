#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace g1_controller {
inline constexpr std::size_t StateDimension = 58;
inline constexpr std::size_t InputDimension = 35;
inline constexpr std::size_t RobotJointCount = 29;
inline constexpr std::size_t ControlledJointCount = 23;

struct ControllerConfig {
  std::string projectRoot;
  std::string logDirectory;
};

struct Observation {
  double rawBackendTime = 0;
  std::array<double, 3> basePosition{};
  std::array<double, 4> baseOrientationWxyz{};
  std::array<double, 3> baseLinearVelocityLocal{};
  std::array<double, 3> baseAngularVelocityLocal{};
  std::array<double, RobotJointCount> jointPositions{};
  std::array<double, RobotJointCount> jointVelocities{};
  std::array<double, 2> measuredContacts{};
};

struct Reference {
  std::uint64_t tick = 0;
  bool publish = false;
  std::size_t targetCount = 0;
  std::array<double, 3> targetTimes{};
  std::array<std::array<double, StateDimension>, 3> targetStates{};
  std::array<std::array<double, InputDimension>, 3> targetInputs{};
  std::size_t eventCount = 0;
  std::array<double, 64> eventTimes{};
  std::array<std::size_t, 65> modes{};
  double terminalTime = 0;
  std::array<double, 8> swingParameters{};
  std::array<double, 4> filteredCommand{};
};

struct Policy {
  std::array<double, StateDimension> state{};
  std::array<double, InputDimension> input{};
  // Keep the original numeric mode representation for exact packet identity.
  double mode = 0;
  double queryTime = 0;
};
using Torque = std::array<double, ControlledJointCount>;
struct ControlResult {
  Policy policy;
  Torque torque;
};

// Public ABI contains no Eigen, OCS2, Pinocchio, MATLAB or ROS types.
// One instance owns its runtime, solver worker and policy cache. Configuration
// of the retained mathematical authority still uses the accepted environment.
class G1Controller {
public:
  G1Controller();
  ~G1Controller();
  G1Controller(const G1Controller &) = delete;
  G1Controller &operator=(const G1Controller &) = delete;

  void initialize(const ControllerConfig &config);
  void setReference(const Reference &reference);
  Policy query(const Observation &observation);
  Torque inverseDynamics(const Policy &policy);
  ControlResult step(const Observation &observation);
  void shutdown() noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace g1_controller
