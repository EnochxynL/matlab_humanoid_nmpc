/**
 * @file
 * @brief 不暴露 MATLAB、ROS、Eigen 或 OCS2 类型的控制核心公共接口。
 * 适配器提交具名数据，核心通过 PImpl 隐藏机器人模型、求解器和策略。
 * 这是 C++ 接口，不是保证跨编译器二进制兼容的 C ABI。
 */
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <memory>
#include <vector>
#include "Diagnostics.h"

#if defined(_WIN32)
#if defined(g1_controller_core_EXPORTS)
#define G1_CONTROLLER_API __declspec(dllexport)
#else
#define G1_CONTROLLER_API __declspec(dllimport)
#endif
#else
#define G1_CONTROLLER_API
#endif

namespace g1_controller {

// 两种问题有不同的状态和输入含义，不能仅凭向量长度互换策略。
enum class MpcFormulation { WholeBody, Centroidal };

struct ProblemDimensions {
  std::size_t state = 0, input = 0, joints = 0;
};

/**
 * @brief 调用者提供完整资源路径；后端不推断工程目录布局。
 */
struct ControllerConfig {
  std::string taskFile;
  std::string referenceFile;
  std::string urdfFile;
  std::string logDirectory;
  // 默认保留现有验收日志；关闭时不创建应用诊断文件或额外诊断模型。
  bool diagnosticsEnabled = true;
  MpcFormulation formulation = MpcFormulation::WholeBody;
};


/**
 * @brief 外部参考与接触时间表的有界容器；节点维度由具体 OCP 决定。
 * 数组为容量，targetCount/eventCount 才是有效数量；模式数为事件数加一。
 * publish 控制本包是否生效；tick 将参考绑定到实际求解的观测时刻。
 */
struct Reference {
  std::uint64_t tick = 0;
  bool publish = false;
  std::size_t targetCount = 0;
  std::array<double, 3> targetTimes{};
  std::array<std::vector<double>, 3> targetStates{};
  std::array<std::vector<double>, 3> targetInputs{};
  std::size_t eventCount = 0;
  std::array<double, 64> eventTimes{};
  std::array<std::size_t, 65> modes{};
  double terminalTime = 0;
  std::array<double, 8> swingParameters{};
  std::array<double, 4> filteredCommand{};
};

/** @brief 显式规划阶段之间传递的身份；由核心创建，调用者不得修改。 */
struct PlannerTick {
  std::uint64_t tick = 0;
  double time = 0;
  double mpcPeriod = 0;
};

/**
 * @brief 一次 MRT 查询的结果，不是整条预测轨迹，也不携带反馈增益矩阵。
 * 状态和输入维度由具体 OCP 决定；不把两种问题的输入含义混用。
 */
struct Policy {
  std::vector<double> state;
  std::vector<double> input;
  // Keep the original numeric mode representation for exact packet identity.
  double mode = 0;
  double queryTime = 0;
};

/**
 * @brief 一个实例直接拥有模型、同步求解器、MRT 及当前策略缓存。
 * 典型调用：initialize → setReference → installOcpObservation → updateMpc → queryPrepared → shutdown。
 * 同 tick 的重复策略查询使用缓存。
 * 接口不承诺可由多个调用线程同时操作；求解在调用线程同步完成。
 */
class G1_CONTROLLER_API G1Controller {
public:
  G1Controller();
  ~G1Controller();
  G1Controller(const G1Controller &) = delete;
  G1Controller &operator=(const G1Controller &) = delete;

/**
 * @brief 加载模型、OCP、数学实现和日志配置；已有实例须先 shutdown。
 */
  void initialize(const ControllerConfig &config);
  ProblemDimensions dimensions() const;
  // 输入为 [广义位置; 广义速度]；centroidal 使用 A(q)*v/m 构造动量。
  std::vector<double> observationFromGeneralizedState(const std::vector<double>& state);
/**
 * @brief 更新外部参考缓存；不会在此调用中直接执行一次 SQP。
 */
  void setReference(const Reference &reference);
/** @brief 外部决定更新时刻；requested=true 同步求解，否则沿用已有策略。 */
  // 调用者负责生成规范时间；这里只登记状态和 [tick, canonical time]。
  PlannerTick installOcpObservation(const std::vector<double>& state, const std::array<double,2>& identity);
  PlannerTick updateMpc(const PlannerTick &tick, bool requested);
  Policy queryPrepared(const PlannerTick &tick);
/** @brief 取走当前批次；重复读取返回空数据，不触发求解或额外诊断计算。 */
  DiagnosticBatch drainDiagnostics();
/**
 * @brief 释放数值后端；允许重复调用，析构也会调用。
 */
  void shutdown() noexcept;

private:
  PlannerTick tickIdentity() const;
  void validateTick(const PlannerTick& token) const;

  // 仅保存内部状态，具体行为由 G1Controller 成员函数实现。
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace g1_controller
