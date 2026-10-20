#include "g1_controller/G1Controller.h"
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/Types.h>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <stdexcept>

using namespace g1_controller;
namespace {
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
template<class Action> void reject(Action action) {
  try { action(); } catch (const std::exception&) { return; }
  throw std::runtime_error("Invalid shared-runtime operation was accepted");
}
bool equal(const Policy& a, const Policy& b) {
  return a.state == b.state && a.input == b.input && a.mode == b.mode && a.queryTime == b.queryTime;
}
}

int main(int argc, char** argv) {
  try {
    require(argc == 3, "Usage: g1_shared_runtime_smoke root evidence-folder");
    const std::filesystem::path root(argv[1]), evidence(argv[2]);
    ControllerConfig config{(root/"config/centroidal/task.info").string(),
                            (root/"config/centroidal/reference.info").string(),
                            (root/"assets/g1_29dof.urdf").string(), evidence.string()};
    config.formulation = MpcFormulation::Centroidal;
    ocs2::vector_t nominal = ocs2::vector_t::Zero(35);
    ocs2::loadData::loadEigenMatrix(config.taskFile, "initialState", nominal);
    const std::vector<double> state(nominal.data(), nominal.data()+nominal.size());
    Reference reference;
    reference.publish = true;
    reference.targetCount = 1;
    reference.targetStates[0] = state;
    reference.targetInputs[0].assign(35, 0.0);
    reference.eventCount = 3;
    reference.eventTimes = {-.5,.5,2.5};
    reference.modes = {3,3,3,3};
    reference.terminalTime = 10;
    reference.swingParameters = {.05,-0.0,.08,-.001,.4,-.15,.3,0};

    // 同一个公共类型可同时持有两种问题，各实例独立拥有求解器及模型。
    G1Controller wholeBody;
    ControllerConfig wb{(root/"config/task.info").string(), (root/"config/reference.info").string(),
                        config.urdfFile, evidence.string(), false};
    wholeBody.initialize(wb);
    require(wholeBody.dimensions().state == 58, "Whole-body dimensions changed");
    std::vector<Policy> withoutDiagnostics;
    for (bool enabled : {false, true}) {
      config.diagnosticsEnabled = enabled;
      G1Controller controller;
      controller.initialize(config);
      const auto dimensions = controller.dimensions();
      require(dimensions.state == 35 && dimensions.input == 35 && dimensions.joints == 23,
              "Centroidal dimensions are not selected by the OCP");
      reject([&]{ controller.initialize(config); });
      reject([&]{ controller.installOcpObservation(state,{1,.002}); });
      reject([&]{ controller.installOcpObservation(std::vector<double>(58),{0,0}); });
      auto malformed = reference;
      malformed.targetStates[0].resize(58);
      reject([&]{ controller.setReference(malformed); });
      size_t query = 0, dynamicsRows = 0;
      for (size_t tick = 0; tick < 3; ++tick) {
        const bool solve = tick != 1;
        if (solve) { reference.tick = tick; controller.setReference(reference); }
        const auto token = controller.installOcpObservation(state,{double(tick),double(tick)*.002});
        require(token.mpcPeriod == 1.0/80.0, "Centroidal MPC period was replaced by whole-body settings");
        reject([&]{ controller.queryPrepared(token); });
        if (tick == 0) reject([&]{ controller.updateMpc(token,false); });
        controller.updateMpc(token,solve);
        controller.updateMpc(token,solve);
        reject([&]{ controller.updateMpc(token,!solve); });
        const auto policy = controller.queryPrepared(token);
        require(equal(policy,controller.queryPrepared(token)), "Duplicate query changed the policy");
        require(policy.state.size()==35 && policy.input.size()==35 && policy.mode==3,
                "Shared runtime returned a whole-body policy shape");
        for (double value : policy.state) require(std::isfinite(value), "Nonfinite centroidal state");
        for (double value : policy.input) require(std::isfinite(value), "Nonfinite centroidal input");
        if (!enabled) withoutDiagnostics.push_back(policy);
        else require(equal(policy,withoutDiagnostics.at(query)), "Diagnostics changed the centroidal policy");
        ++query;
        const auto batch = controller.drainDiagnostics();
        require(batch.snapshot.empty(), "Centroidal diagnostics produced a whole-body snapshot");
        for (const auto& table : batch.tables) {
          require(table.name != "structural_id", "Centroidal diagnostics used whole-body inverse dynamics");
          if (table.name == "structural_dynamics") dynamicsRows += table.rows.size();
        }
        if (!enabled) require(batch.tables.empty(), "Disabled diagnostics produced tables");
      }
      if (enabled) require(dynamicsRows > 0, "Generic OCP diagnostics were not evaluated");
      const auto failed = controller.installOcpObservation(state,{3,.006});
      reject([&]{ controller.updateMpc(failed,true); }); // 没有 tick 3 的参考。
      reject([&]{ controller.updateMpc(failed,false); });
      reject([&]{ controller.queryPrepared(failed); });
      controller.shutdown();
      controller.shutdown();
      controller.initialize(config);
      require(controller.dimensions().state == 35, "Reinitialization lost formulation");
      reference.tick = 0;
      controller.setReference(reference);
      const auto restarted = controller.installOcpObservation(state,{0,0});
      controller.updateMpc(restarted,true);
      require(controller.queryPrepared(restarted).state.size()==35, "Reinitialized solver did not recover");
    }
    G1Controller invalid;
    config.formulation = static_cast<MpcFormulation>(99);
    reject([&]{ invalid.initialize(config); });
    std::filesystem::create_directories(evidence);
    std::ofstream samples(evidence/"policy.csv");
    samples << std::setprecision(17) << "observation_time,query_time,mode";
    for (size_t i=0;i<35;++i) samples << ",state" << i;
    for (size_t i=0;i<35;++i) samples << ",input" << i;
    samples << '\n';
    for (size_t tick=0;tick<withoutDiagnostics.size();++tick) {
      const auto& policy = withoutDiagnostics[tick];
      samples << double(tick)*.002 << ',' << policy.queryTime << ',' << policy.mode;
      for (double value : policy.state) samples << ',' << value;
      for (double value : policy.input) samples << ',' << value;
      samples << '\n';
    }
    std::ofstream report(evidence/"acceptance.json");
    report << "{\"passed\":true,\"coexisting_formulations\":2,\"diagnostics_exact_equal\":true,"
              "\"dimension_guards\":true,\"stage_guards\":true,\"failure_recovery\":true,"
              "\"physical_simulation\":false}\n";
    require(bool(report) && bool(samples), "Cannot write shared-runtime evidence");
    std::cout << "SHARED_RUNTIME_PASS\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
