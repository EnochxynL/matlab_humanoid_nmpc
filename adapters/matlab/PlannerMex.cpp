/**
 * @file
 * @brief MATLAB 与独立 G1 控制核心之间的薄适配层。
 * 只负责 mxArray 校验、固定长度向量编解码、实例生命周期及异常转换。
 * 机器人数学、求解调度和策略缓存均由 G1Controller 及其内部实现负责。
 * 阅读顺序：本文件 → G1Controller.h → G1Controller.cpp。
 * 下文数组偏移均为 C++ 从 0 开始的索引；MATLAB 索引需加 1。
 */
#include "G1Controller.h"
#include "mex.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

namespace {
using g1_controller::G1Controller;
/**
 * @brief 当前 MEX 模块持有的唯一控制实例；跨多次 MATLAB 调用保留。
 */
std::unique_ptr<G1Controller> controller;

/**
 * @brief 读取 MATLAB 字符向量，并释放 mxArrayToString 分配的临时内存。
 */
std::string readString(const mxArray *argument) {
  if (!mxIsChar(argument))
    throw std::runtime_error("Expected character vector");
  char *text = mxArrayToString(argument);
  std::string result(text);
  mxFree(text);
  return result;
}

/**
 * @brief 校验实数 double 数组的元素总数和有限性，返回借用指针。
 * 不强制行/列形状；指针仅在本次 MEX 调用期间使用，不转移所有权。
 */
const double *readDoubles(const mxArray *argument, std::size_t count) {
  if (!mxIsDouble(argument) || mxIsComplex(argument) ||
      mxGetNumberOfElements(argument) != count)
    throw std::runtime_error("Invalid numeric argument dimensions");
  const double *values = mxGetPr(argument);
  for (std::size_t i = 0; i < count; ++i)
    if (!std::isfinite(values[i]))
      throw std::runtime_error("Nonfinite argument");
  return values;
}

/**
 * @brief 获取已初始化实例；未 init 时抛异常，随后转换为 MATLAB 错误。
 * 我觉得不如叫 get_controller()
 */
G1Controller &runtime() {
  if (!controller)
    throw std::runtime_error("Initialize MEX first");
  return *controller;
}

/**
 * @brief 关闭核心并释放实例；用于 close 命令、初始化失败和 MEX 退出清理。
 */
void closeController() {
  if (controller)
    controller->shutdown();
  controller.reset();
}

// These offsets are the unchanged MATLAB transport protocol. No robot math
// or solver scheduling belongs in this adapter.
/**
 * @brief 将 74 元素观测包转换为具名结构。
 * 布局：时间[0]、世界位置[1:3]、wxyz 四元数[4:7]、局部线/角速度[8:13]、
 * 29 关节位置[14:42]、29 关节速度[43:71]、接触标志[72:73]。
 * 这里只搬运数据，不做坐标转换或关节重排。
 */
g1_controller::Observation decodeObservation(const double *a) {
  g1_controller::Observation o;
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

/**
 * @brief 解码 428 元素参考包；publish 为假时直接返回。
 * 包含 tick、至多 3 个 58/35 维目标节点、至多 64 个事件和 65 个模式，
 * 以及终止时间、摆腿参数和滤波命令；实际计数由核心继续校验。
 */
g1_controller::Reference decodeReference(const double *a) {
  g1_controller::Reference r;
  r.publish = a[1] != 0;
  if (!r.publish)
    return r;
  r.tick = static_cast<std::uint64_t>(a[0]);
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

/**
 * @brief 解码 query 返回的 95 元素策略：状态58、输入35、模式1、查询时间1。
 */
g1_controller::Policy decodePolicy(const double *a) {
  g1_controller::Policy p;
  std::copy_n(a, 58, p.state.begin());
  std::copy_n(a + 58, 35, p.input.begin());
  p.mode = a[93];
  p.queryTime = a[94];
  return p;
}

/**
 * @brief 创建 MATLAB 拥有的 95×1 策略向量，供后续 inverse_dynamics 原样回传。
 */
mxArray *encodePolicy(const g1_controller::Policy &p) {
  mxArray *output = mxCreateDoubleMatrix(95, 1, mxREAL);
  double *a = mxGetPr(output);
  std::copy(p.state.begin(), p.state.end(), a);
  std::copy(p.input.begin(), p.input.end(), a + 58);
  a[93] = p.mode;
  a[94] = p.queryTime;
  return output;
}

/**
 * @brief 编码兼容 step 输出：状态58、输入35、前馈力矩23、模式1、查询时间1。
 * 该 118 元素结果不包含 JointDrive 最终叠加的 PD 反馈力矩。
 */
mxArray *encodeResult(const g1_controller::ControlResult &r) {
  mxArray *output = mxCreateDoubleMatrix(118, 1, mxREAL);
  double *a = mxGetPr(output);
  std::copy(r.policy.state.begin(), r.policy.state.end(), a);
  std::copy(r.policy.input.begin(), r.policy.input.end(), a + 58);
  std::copy(r.torque.begin(), r.torque.end(), a + 93);
  a[116] = r.policy.mode;
  a[117] = r.policy.queryTime;
  return output;
}
g1_controller::PlannerTick decodeTick(const double* a) {
  if (a[0] < 0 || a[0] > 9007199254740992.0 || std::floor(a[0]) != a[0])
    throw std::runtime_error("Invalid planner tick");
  return {static_cast<std::uint64_t>(a[0]), a[1], a[2]};
}
mxArray* encodeTick(const g1_controller::PlannerTick& tick) {
  auto* result = mxCreateDoubleMatrix(3, 1, mxREAL);
  auto* a = mxGetPr(result);
  a[0] = static_cast<double>(tick.tick); a[1] = tick.time; a[2] = tick.mpcPeriod;
  return result;
}
} // namespace

/**
 * @brief MEX 命令分发入口，保持既有输入输出协议。
 * init(root,folder)：创建核心并 mexLock，防止运行中卸载；close：关闭并解锁。
 * reference(r428)：提交参考；query(o74) → p95：兼容的组合更新/查询。
 * prepare_observation(o74) → tick3：装填观测，不求解。
 * update_mpc(tick3,request) → tick3：校验外部更新判断，等待必要求解完成。
 * query_policy(tick3) → p95：只查询已准备好的策略，不触发求解。
 * inverse_dynamics(p95) → tau23：计算该策略的前馈力矩。
 * step(o74) → r118：依次执行 query 和 inverse_dynamics。
 * 标准 C++ 异常统一转换为 G1:PlannerMex 错误；此处不实现控制算法。
 */
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  try {
    if (nrhs < 1)
      throw std::runtime_error("Expected init/reference/step/close command");
    const std::string command = readString(prhs[0]);
    if (command == "init") {
      if (nrhs != 3 || nlhs)
        throw std::runtime_error("init(root,folder)");
      static bool registered = false;
      if (!registered) {
        mexAtExit(closeController);
        registered = true;
      }
      const g1_controller::ControllerConfig config{readString(prhs[1]),
                                                   readString(prhs[2])};
      if (controller)
        throw std::runtime_error(
            "Close existing MEX controller before initialization");
      controller = std::make_unique<G1Controller>();
      try {
        controller->initialize(config);
      } catch (...) {
        closeController();
        throw;
      }
      mexLock();
    } else if (command == "close") {
      closeController();
      if (mexIsLocked())
        mexUnlock();
    } else if (command == "reference") {
      if (nrhs != 2)
        throw std::runtime_error("reference requires 428 values");
      const double *values = readDoubles(prhs[1], 428);
      runtime().setReference(decodeReference(values));
    } else if (command == "prepare_ocp_observation") {
      if (nrhs != 3 || nlhs != 1) throw std::runtime_error("prepare_ocp_observation requires o74, x58 and one output");
      const auto raw = decodeObservation(readDoubles(prhs[1], 74));
      const auto* values = readDoubles(prhs[2], 58);
      std::array<double, 58> state;
      std::copy_n(values, 58, state.begin());
      plhs[0] = encodeTick(runtime().prepareOcpObservation(raw, state));
    } else if (command == "convert_observation") {
      if (nrhs != 2 || nlhs != 1) throw std::runtime_error("convert_observation requires o74 and one output");
      const auto state = runtime().convertObservation(decodeObservation(readDoubles(prhs[1], 74)));
      plhs[0] = mxCreateDoubleMatrix(58, 1, mxREAL);
      std::copy(state.begin(), state.end(), mxGetPr(plhs[0]));
    } else if (command == "prepare_observation") {
      if (nrhs != 2 || nlhs != 1) throw std::runtime_error("prepare_observation requires o74 and one output");
      plhs[0] = encodeTick(runtime().prepareObservation(decodeObservation(readDoubles(prhs[1], 74))));
    } else if (command == "update_mpc") {
      if (nrhs != 3 || nlhs != 1) throw std::runtime_error("update_mpc requires tick3, request and one output");
      const auto tick = decodeTick(readDoubles(prhs[1], 3));
      const double request = readDoubles(prhs[2], 1)[0];
      if (request != 0 && request != 1) throw std::runtime_error("MPC request must be zero or one");
      plhs[0] = encodeTick(runtime().updateMpc(tick, request != 0));
    } else if (command == "query_policy") {
      if (nrhs != 2 || nlhs != 1) throw std::runtime_error("query_policy requires tick3 and one output");
      plhs[0] = encodePolicy(runtime().queryPrepared(decodeTick(readDoubles(prhs[1], 3))));
    } else if (command == "query") {
      if (nrhs != 2 || nlhs != 1)
        throw std::runtime_error("query requires 74 values and one output");
      const double *values = readDoubles(prhs[1], 74);
      plhs[0] = encodePolicy(runtime().query(decodeObservation(values)));
    } else if (command == "inverse_dynamics") {
      if (nrhs != 2 || nlhs != 1)
        throw std::runtime_error(
            "inverse_dynamics requires queried 95-value policy and one output");
      const double *values = readDoubles(prhs[1], 95);
      const auto torque = runtime().inverseDynamics(decodePolicy(values));
      plhs[0] = mxCreateDoubleMatrix(23, 1, mxREAL);
      std::copy(torque.begin(), torque.end(), mxGetPr(plhs[0]));
    } else if (command == "step") {
      if (nrhs != 2 || nlhs != 1)
        throw std::runtime_error("step requires 74 values and one output");
      const double *values = readDoubles(prhs[1], 74);
      plhs[0] = encodeResult(runtime().step(decodeObservation(values)));
    } else {
      throw std::runtime_error("Unknown MEX command");
    }
  } catch (const std::exception &error) {
    mexErrMsgIdAndTxt("G1:PlannerMex", "%s", error.what());
  }
}
