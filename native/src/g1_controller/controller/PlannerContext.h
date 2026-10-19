/**
 * @file
 * @brief 控制核心的对象装配入口：模型/OCP → SQP MPC → 参考与诊断。
 * 本文件主要管理依赖关系，不是 SQP 算法实现。TASK5A 名称是保留的参考接入配置。
 */
#pragma once
#include "controller/Ocs2Backend.h"
#include "controller/data/diagnostics/StructuralObserver.h"
#include "controller/data/diagnostics/TimeLog.h"
#include "controller/reference/ExternalReferenceManager.h"
#include "controller/data/time/CanonicalControlTime.h"
#include "g1_controller/G1Controller.h"

#include <humanoid_wb_mpc/WBMpcInterface.h>

#include <cstdint>
#include <memory>
#include <string>
namespace g1_controller {

/**
 * @brief 拥有一套完整规划上下文。成员按声明顺序初始化、按逆序析构。
 * interface 提供完整 OCP 与初始化器，mpc 直接使用其 C++ 实现。
 */
struct PlannerContext {
  ocs2::humanoid::WBMpcInterface interface;
  ocs2::SqpMpc mpc;
  std::shared_ptr<TimeLog> log;
  std::shared_ptr<ExternalReferenceManager> external;
  std::shared_ptr<diagnostics::StructuralObserver> structure;
  ReferencePacket currentPacket;
  bool migrated;
  uint64_t expectedSession = 0;
  /**
   * @brief 构建 WBMpcInterface，直接以原 C++ OCP 和初始化器创建 SqpMpc。
   * 随后绑定 Simulink 外部参考：每次求解都检查 session 与实际观测 tick 一致。
   * 日志和结构观察器挂到求解同步接口。
   */
  PlannerContext(const ControllerConfig &config);

  /**
   * @brief 为同步数值后端绑定诊断回调；调度由外部负责。
   */
  void configure(Ocs2Backend &c);
};

} // namespace g1_controller
