#pragma once
#include "g1_controller/ModelTypes.h"
#include <memory>
#include <string>

#if defined(_WIN32)
#if defined(g1_dynamics_core_EXPORTS)
#define G1_DYNAMICS_API __declspec(dllexport)
#else
#define G1_DYNAMICS_API __declspec(dllimport)
#endif
#else
#define G1_DYNAMICS_API
#endif

namespace g1_controller {
/** 独立逆动力学模型；不创建 MPC、MRT、参考管理器或诊断对象。 */
class G1_DYNAMICS_API InverseDynamics final {
 public:
  InverseDynamics(const std::string& taskFile, const std::string& urdfFile);
  ~InverseDynamics();
  InverseDynamics(const InverseDynamics&) = delete;
  InverseDynamics& operator=(const InverseDynamics&) = delete;
  Torque compute(const std::array<double, StateDimension>& state,
                 const std::array<double, InputDimension>& input);
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace g1_controller
