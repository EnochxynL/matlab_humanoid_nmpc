#include "g1_controller/InverseDynamics.h"
#include <humanoid_common_mpc/pinocchio_model/createPinocchioModel.h>
#include <humanoid_wb_mpc/dynamics/DynamicsHelperFunctions.h>
#include <humanoid_wb_mpc/common/WBAccelMpcRobotModel.h>
#include <algorithm>
#include <stdexcept>

namespace g1_controller {
struct InverseDynamics::Impl {
  ocs2::humanoid::ModelSettings settings;
  ocs2::PinocchioInterface pinocchio;
  ocs2::humanoid::WBAccelMpcRobotModel<ocs2::scalar_t> robotModel;
  Impl(const std::string& taskFile, const std::string& urdfFile)
      : settings(taskFile, urdfFile, "wb_mpc_", true),
        pinocchio(ocs2::humanoid::createCustomPinocchioInterface(taskFile, urdfFile, settings)),
        robotModel(settings) {}
};

InverseDynamics::InverseDynamics(const std::string& taskFile, const std::string& urdfFile)
    : impl_(std::make_unique<Impl>(taskFile, urdfFile)) {}
InverseDynamics::~InverseDynamics() = default;

Torque InverseDynamics::compute(const std::array<double, StateDimension>& state,
                               const std::array<double, InputDimension>& input) {
  const Eigen::Map<const ocs2::vector_t> x(state.data(), state.size());
  const Eigen::Map<const ocs2::vector_t> u(input.data(), input.size());
  if (!x.allFinite() || !u.allFinite())
    throw std::runtime_error("Nonfinite inverse dynamics inputs");
  const auto result = ocs2::humanoid::computeJointTorques<ocs2::scalar_t>(
      x, u, impl_->pinocchio, impl_->robotModel);
  if (result.size() != ControlledJointCount || !result.allFinite())
    throw std::runtime_error("Invalid inverse dynamics output");
  Torque torque;
  std::copy_n(result.data(), torque.size(), torque.begin());
  return torque;
}
}  // namespace g1_controller
