#pragma once

#include "controller/Ocs2Backend.h"
#include "controller/ExternalReferenceManager.h"
#include "g1_controller/G1Controller.h"

#include <humanoid_wb_mpc/WBMpcInterface.h>
#include <ocs2_sqp/SqpMpc.h>

#include <cstdint>
#include <memory>

namespace g1_controller {
namespace diagnostics { class Diagnostics; }

/**
 * @brief 装配并拥有一套 OCP、MPC 和外部参考接入，可选挂接诊断。
 * 不生成参考、不安排求解时刻；这些由 MATLAB/Simulink 负责。
 * G1Controller 先销毁使用本上下文的 Ocs2Backend，再销毁本对象。
 */
struct PlannerContext {
  explicit PlannerContext(const ControllerConfig& config);
  ~PlannerContext();

  // 回调捕获本对象地址，禁止复制或移动上下文。
  PlannerContext(const PlannerContext&) = delete;
  PlannerContext& operator=(const PlannerContext&) = delete;

  void attachBackendDiagnostics(Ocs2Backend& backend);

  // 按依赖顺序声明：MPC 先于诊断销毁，机器人/OCP 接口最后销毁。
  ocs2::humanoid::WBMpcInterface interface;
  std::unique_ptr<diagnostics::Diagnostics> diagnostics;
  PacketManager packets;
  ocs2::SqpMpc mpc;

 private:
  void installReferenceManager();

  std::shared_ptr<ExternalReferenceManager> referenceManager_;
};

} // namespace g1_controller
