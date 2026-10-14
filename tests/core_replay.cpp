// Integration acceptance: only public core API + saved real observations.
#include "G1Controller.h"
#include <algorithm>
#include <array>
#include <boost/property_tree/json_parser.hpp>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <stdexcept>
using namespace g1_controller;

Observation observation(const double *a) {
  Observation o;
  o.rawBackendTime = a[0];
  std::copy_n(a + 1, 3, o.basePosition.begin());
  std::copy_n(a + 4, 4, o.baseOrientationWxyz.begin());
  std::copy_n(a + 8, 3, o.baseLinearVelocityLocal.begin());
  std::copy_n(a + 11, 3, o.baseAngularVelocityLocal.begin());
  std::copy_n(a + 14, 29, o.jointPositions.begin());
  std::copy_n(a + 43, 29, o.jointVelocities.begin());
  std::copy_n(a + 72, 2, o.measuredContacts.begin());
  return o;
}
Reference reference(const double *a) {
  Reference r;
  r.tick = static_cast<std::uint64_t>(a[0]);
  r.publish = a[1] != 0;
  r.targetCount = static_cast<std::size_t>(a[2]);
  std::copy_n(a + 3, 3, r.targetTimes.begin());
  for (std::size_t k = 0; k < 3; ++k) {
    std::copy_n(a + 6 + 58 * k, 58, r.targetStates[k].begin());
    std::copy_n(a + 180 + 35 * k, 35, r.targetInputs[k].begin());
  }
  r.eventCount = static_cast<std::size_t>(a[285]);
  std::copy_n(a + 286, 64, r.eventTimes.begin());
  for (std::size_t k = 0; k < 65; ++k)
    r.modes[k] = static_cast<std::size_t>(a[350 + k]);
  r.terminalTime = a[415];
  std::copy_n(a + 416, 8, r.swingParameters.begin());
  std::copy_n(a + 424, 4, r.filteredCommand.begin());
  return r;
}
std::array<double, 118> result(const ControlResult &r) {
  std::array<double, 118> a;
  std::copy(r.policy.state.begin(), r.policy.state.end(), a.begin());
  std::copy(r.policy.input.begin(), r.policy.input.end(), a.begin() + 58);
  std::copy(r.torque.begin(), r.torque.end(), a.begin() + 93);
  a[116] = r.policy.mode;
  a[117] = r.policy.queryTime;
  return a;
}
int main(int argc, char **argv) {
  try {
    if (argc != 5 && argc != 6)
      throw std::runtime_error(
          "core_replay root fixture log_folder split|step [measure]");
    const std::string root = argv[1], folder = argv[3], mode = argv[4];
    boost::property_tree::ptree env;
    boost::property_tree::read_json(
        root + "/config/controller_environment.json", env);
    for (const auto &x : env)
      setenv(x.first.c_str(), x.second.get_value<std::string>().c_str(), 1);
    setenv("TASK5A_AUTHORITY", "SIMULINK_REFERENCE", 1);
    setenv("TASK5A_PACKET_FILE", "IN_PROCESS_ONLY", 1);
    std::ifstream input(argv[2], std::ios::binary);
    std::uint64_t rows = 0;
    input.read(reinterpret_cast<char *>(&rows), 8);
    if (!input || rows != 2401)
      throw std::runtime_error("Expected complete 2401-tick fixture");
    G1Controller core;
    core.initialize({root, folder});
    const bool measure = argc == 6 && std::string(argv[5]) == "measure";
    std::array<double, 118> maxima{};
    std::array<std::uint64_t, 118> maximumTicks{};
    double maximum = 0;
    std::size_t nonidenticalBits = 0;
    for (std::uint64_t k = 0; k < rows; ++k) {
      std::array<double, 620> a{};
      input.read(reinterpret_cast<char *>(a.data()), sizeof(a));
      if (!input)
        throw std::runtime_error("Truncated fixture");
      const auto obs = observation(a.data());
      core.setReference(reference(a.data() + 74));
      ControlResult control;
      if (mode == "step")
        control = core.step(obs);
      else {
        control.policy = core.query(obs);
        if (k == 0) {
          auto bad = control.policy;
          bad.state[0] += .01;
          bool rejected = false;
          try {
            core.inverseDynamics(bad);
          } catch (const std::exception &) {
            rejected = true;
          }
          if (!rejected)
            throw std::runtime_error("Altered policy accepted");
          auto next = obs;
          next.rawBackendTime = .002;
          rejected = false;
          try {
            core.query(next);
          } catch (const std::exception &) {
            rejected = true;
          }
          if (!rejected)
            throw std::runtime_error("Unfinished tick accepted");
        }
        control.torque = core.inverseDynamics(control.policy);
        if (k == 0 && core.inverseDynamics(control.policy) != control.torque)
          throw std::runtime_error("Repeated ID changed");
      }
      auto actual = result(control);
      const double *expected = a.data() + 502;
      for (std::size_t j = 0; j < 118; ++j) {
        if (!std::isfinite(actual[j]))
          throw std::runtime_error("Nonfinite result");
        const double difference = std::abs(actual[j] - expected[j]);
        if (difference > maxima[j]) { maxima[j] = difference; maximumTicks[j] = k; }
        maximum = std::max(maximum, difference);
        if (std::memcmp(&actual[j], &expected[j], sizeof(double)))
          ++nonidenticalBits;
      }
      if (actual[116] != expected[116] || actual[117] != expected[117])
        throw std::runtime_error("Mode/time identity changed");
      if (!measure && maximum != 0)
        throw std::runtime_error("Nonzero control difference at tick " +
                                 std::to_string(k));
    }
    core.shutdown();
    std::ofstream out(folder + (measure ? "/comparison.json" : "/acceptance.json"));
    out << "{\"completed\":true,\"ticks\":2401,\"mode\":\"" << mode
        << "\",\"exact_equal\":" << (maximum == 0 ? "true" : "false")
        << ",\"maximum_absolute_error\":" << std::setprecision(17) << maximum
        << ",\"different_double_bit_patterns\":" << nonidenticalBits
        << ",\"physical_simulation\":false,\"field_maxima\":[";
    for (std::size_t j=0;j<118;++j) { if(j) out << ','; out << maxima[j]; }
    out << "],\"field_maximum_ticks\":[";
    for (std::size_t j=0;j<118;++j) { if(j) out << ','; out << maximumTicks[j]; }
    out << "]}\n";
    std::cout << (measure ? "CORE_REPLAY_MEASURE_COMPLETE" : "CORE_REPLAY_PASS")
              << " ticks=2401 max_abs=" << maximum << " different_bits="
              << nonidenticalBits << '\n';
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
