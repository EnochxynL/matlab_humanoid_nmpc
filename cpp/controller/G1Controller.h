/**
 * @file
 * @brief 不暴露 MATLAB、ROS、Eigen 或 OCS2 类型的控制核心公共接口。
 * 适配器提交具名数据，核心通过 PImpl 隐藏机器人模型、求解器和线程。
 * 这是 C++ 接口，不是保证跨编译器二进制兼容的 C ABI。
 */
#pragma once

#include "g1_controller/Types.h"
#include <memory>

namespace g1_controller {
/**
 * @brief 一个实例拥有一套运行时、求解工作线程及当前策略缓存。
 * 典型调用：initialize → setReference → query → inverseDynamics → shutdown。
 * step 是 query 与 inverseDynamics 的组合；同 tick 的重复查询使用缓存。
 * 接口不承诺可由多个调用线程同时操作；内部工作线程同步由实现负责。
 */
class G1Controller {
public:
  G1Controller();
  ~G1Controller();
  G1Controller(const G1Controller &) = delete;
  G1Controller &operator=(const G1Controller &) = delete;

/**
 * @brief 加载模型、OCP、数学实现和日志配置；已有实例须先 shutdown。
 */
  void initialize(const ControllerConfig &config);
/**
 * @brief 更新外部参考缓存；不会在此调用中直接执行一次 SQP。
 */
  void setReference(const Reference &reference);
/**
 * @brief 接收连续物理 tick 的观测；到 MPC 截止时间时等待求解，再查询 t+5ms 策略。
 */
  Policy query(const Observation &observation);
/**
 * @brief 只接受当前 tick 最近查询的原样策略；拒绝陈旧或被修改的数据。
 * 完成此步骤后才允许推进下一物理 tick，重复调用返回已缓存力矩。
 */
  Torque inverseDynamics(const Policy &policy);
/**
 * @brief 按顺序执行 query 与 inverseDynamics，语义与拆分调用一致。
 */
  ControlResult step(const Observation &observation);
/**
 * @brief 释放控制器并等待其工作线程结束；允许重复调用，析构也会调用。
 */
  void shutdown() noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace g1_controller
