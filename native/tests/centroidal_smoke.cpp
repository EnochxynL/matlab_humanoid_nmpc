#include <humanoid_centroidal_mpc/CentroidalMpcInterface.h>
#include <humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_sqp/SqpMpc.h>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

// 验证第二种 OCP 的真实求解链；不经过全身控制器或逆动力学接口。
int main(int argc, char** argv) {
  try {
    if (argc != 3) throw std::runtime_error("Usage: g1_centroidal_smoke project-root evidence-folder");
    const std::filesystem::path root(argv[1]), evidence(argv[2]);
    ocs2::humanoid::CentroidalMpcInterface interface(
        (root / "config/centroidal/task.info").string(),
        (root / "assets/g1_29dof.urdf").string(),
        (root / "config/centroidal/reference.info").string());
    const auto& model = interface.getMpcRobotModel();
    const auto& info = interface.getCentroidalModelInfo();
    if (model.getStateDim() != 35 || model.getInputDim() != 35 || model.getJointDim() != 23)
      throw std::runtime_error("Unexpected G1 centroidal dimensions");
    const auto reference = interface.getSwitchedModelReferenceManagerPtr();
    ocs2::SqpMpc mpc(interface.mpcSettings(), interface.sqpSettings(),
                     interface.getOptimalControlProblem(), interface.getInitializer());
    mpc.getSolverPtr()->setReferenceManager(reference);
    ocs2::MPC_MRT_Interface mrt(mpc);
    ocs2::SystemObservation observation;
    observation.state = interface.getInitialState();
    observation.input = ocs2::humanoid::weightCompensatingInput(info.robotMass, {true, true}, model);
    const auto fromPinocchio = ocs2::humanoid::weightCompensatingInput(interface.getPinocchioInterface(), {true, true}, model);
    if (observation.input != fromPinocchio)
      throw std::runtime_error("Mass and Pinocchio weight compensation differ");
    for (double mass : {20.0,40.0,20.0}) {
      const auto input = ocs2::humanoid::weightCompensatingInput(mass, {true,true}, model);
      if (input[2]+input[8] != mass*9.81)
        throw std::runtime_error("Weight compensation reused another instance's mass");
    }
    observation.mode = 3;
    observation.time = 0;
    if (!observation.state.allFinite() || info.robotMass <= 0)
      throw std::runtime_error("Invalid centroidal model or initial state");
    mrt.resetMpcNode({{0.0}, {observation.state}, {observation.input}});
    std::filesystem::create_directories(evidence);
    std::ofstream samples(evidence / "policy.csv");
    samples << std::setprecision(17) << "solve_time,query_time,mode";
    for (size_t i = 0; i < model.getStateDim(); ++i) samples << ",state" << i;
    for (size_t i = 0; i < model.getInputDim(); ++i) samples << ",input" << i;
    samples << '\n';
    size_t queries = 0;
    // 固定站立观测，多次更新以覆盖热启动；不是物理闭环或行走验收。
    for (size_t solve = 0; solve < 3; ++solve) {
      observation.time = double(solve) / interface.mpcSettings().mpcDesiredFrequency_;
      reference->setMeasuredMode(observation.mode);
      mrt.setCurrentObservation(observation);
      mrt.advanceMpc();
      if (!mrt.updatePolicy() || !mrt.initialPolicyReceived())
        throw std::runtime_error("Centroidal solve did not publish a policy");
      const auto& policy = mrt.getPolicy();
      if (policy.timeTrajectory_.empty() || policy.stateTrajectory_.size() != policy.timeTrajectory_.size())
        throw std::runtime_error("Malformed centroidal policy");
      const auto performance = mpc.getSolverPtr()->getPerformanceIndeces();
      if (!std::isfinite(performance.cost) || !std::isfinite(performance.equalityConstraintsSSE))
        throw std::runtime_error("Nonfinite centroidal solver result");
      for (double offset : {0.005, 0.007}) {
        ocs2::vector_t state, input;
        size_t mode;
        const double time = observation.time + offset;
        if (time < policy.timeTrajectory_.front() || time > policy.timeTrajectory_.back())
          throw std::runtime_error("Query outside centroidal policy horizon");
        mrt.evaluatePolicy(time, observation.state, state, input, mode);
        if (state.size() != 35 || input.size() != 35 || !state.allFinite() || !input.allFinite() || mode != 3)
          throw std::runtime_error("Invalid centroidal policy query");
        samples << observation.time << ',' << time << ',' << mode;
        for (double value : state) samples << ',' << value;
        for (double value : input) samples << ',' << value;
        samples << '\n';
        ++queries;
      }
    }
    std::ofstream report(evidence / "acceptance.json");
    report << "{\"passed\":true,\"state_dimension\":35,\"input_dimension\":35,"
              "\"solves\":3,\"queries\":" << queries << ",\"physical_simulation\":false}\n";
    if (!samples || !report) throw std::runtime_error("Cannot write centroidal smoke evidence");
    std::cout << "CENTROIDAL_SMOKE_PASS solves=3 queries=" << queries << '\n';
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
