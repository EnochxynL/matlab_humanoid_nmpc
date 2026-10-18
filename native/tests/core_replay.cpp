// Integration acceptance: only public core API + saved real observations.
#include "g1_controller/G1Controller.h"
#include <algorithm>
#include <array>
#include <boost/property_tree/json_parser.hpp>
#include <cmath>
#include <cstdlib>
#ifdef _WIN32
#include <windows.h>
static int setenv(const char* name, const char* value, int overwrite) {
  if (!overwrite && GetEnvironmentVariableA(name,nullptr,0)) return 0;
  if (_putenv_s(name, value) != 0) return -1;
  return SetEnvironmentVariableA(name,value) ? 0 : -1;
}
#endif
#include <cstring>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <stdexcept>
using namespace g1_controller;
struct ControlResult { Policy policy; Torque torque; };

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
          "core_replay root fixture log_folder installed [measure]; requires fixture.states.bin");
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
    if(mode!="installed") throw std::runtime_error("Use installed mode; raw-observation core API retired");
    std::ifstream states(argv[2]+std::string(".states.bin"),std::ios::binary);
    if(!states) throw std::runtime_error("Missing preconverted OCP state fixture");
    double epoch=0, period=1./60.;size_t nextSlot=1;
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
      std::array<double,58> state;
      states.read(reinterpret_cast<char*>(state.data()),sizeof(state));
      if(!states) throw std::runtime_error("Truncated OCP state fixture");
      core.setReference(reference(a.data()+74));
      const double t=double(k)*.002;
      auto token=core.installOcpObservation(state,{double(k),t,a[0]});
      bool due=k==0 || t+1e-12>=epoch+nextSlot*period;
      if(k==0){epoch=t;period=token.mpcPeriod;}
      else if(due){while(epoch+(nextSlot+1)*period<=t+1e-12)++nextSlot;++nextSlot;}
      core.updateMpc(token,due);
      ControlResult control;
      control.policy=core.queryPrepared(token);
      if(k==0){
        auto bad=control.policy;bad.state[0]+=.01;
        bool rejected=false;try{core.inverseDynamics(bad);}catch(const std::exception&){rejected=true;}
        if(!rejected) throw std::runtime_error("Altered policy accepted");
      }
      control.torque=core.inverseDynamics(control.policy);
      if(k==0 && core.inverseDynamics(control.policy)!=control.torque)
        throw std::runtime_error("Repeated ID changed");
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
