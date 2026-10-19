/**
 * @file
 * @brief MATLAB 与独立 G1 控制核心之间的薄适配层。
 * 只负责 mxArray 校验、固定长度向量编解码、实例生命周期及异常转换。
 * MATLAB/Simulink 决定求解时刻；G1Controller 保管数值对象和策略缓存。
 * 阅读顺序：本文件 → G1Controller.h → G1Controller.cpp。
 * 下文数组偏移均为 C++ 从 0 开始的索引；MATLAB 索引需加 1。
 */
#include "g1_controller/G1Controller.h"
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
 * @brief 解码 query_policy 返回的 95 元素策略：状态58、输入35、模式1、查询时间1。
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
 * @brief MEX 命令分发入口，仅暴露当前分阶段运行接口。
 * init(config)：使用 MATLAB 提供的路径创建核心并 mexLock；close：关闭并解锁。
 * install_reference(r428)：安装已提取参考。
 * install_observation(x58, identity3) → tick3：登记独立提取的观测，不求解。
 * update_mpc(tick3,request) → tick3：按外部请求同步求解或沿用策略。
 * query_policy(tick3) → p95：只查询已准备好的策略，不触发求解。
 * inverse_dynamics(p95) → tau23：计算该策略的前馈力矩。
 * 标准 C++ 异常统一转换为 G1:PlannerMex 错误；此处不实现控制算法。
 */
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  try {
    if (nrhs < 1)
      throw std::runtime_error("Expected planner command");
    const std::string command = readString(prhs[0]);
    if (command == "init") {
      if (nrhs != 2 || nlhs || !mxIsStruct(prhs[1]) || mxGetNumberOfElements(prhs[1]) != 1)
        throw std::runtime_error("init(config): expected a scalar configuration struct");
      static bool registered = false;
      if (!registered) {
        mexAtExit(closeController);
        registered = true;
      }
      const auto field = [&](const char* name) {
        const auto* value = mxGetField(prhs[1], 0, name);
        if (!value) throw std::runtime_error(std::string("Missing configuration field: ") + name);
        return readString(value);
      };
      const g1_controller::ControllerConfig config{field("taskFile"), field("referenceFile"),
                                                  field("urdfFile"), field("logDirectory")};
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
    } else if (command == "install_reference") {
      if (nrhs != 2)
        throw std::runtime_error("install_reference requires 428 values");
      const double *values = readDoubles(prhs[1], 428);
      runtime().setReference(decodeReference(values));
    } else if (command == "install_observation") {
      if(nrhs!=3 || nlhs!=1) throw std::runtime_error("install_observation requires x58, identity3 and one output");
      std::array<double,58> state;
      std::array<double,3> identity;
      std::copy_n(readDoubles(prhs[1],58),58,state.begin());
      std::copy_n(readDoubles(prhs[2],3),3,identity.begin());
      plhs[0]=encodeTick(runtime().installOcpObservation(state,identity));
    } else if (command == "update_mpc") {
      if (nrhs != 3 || nlhs != 1) throw std::runtime_error("update_mpc requires tick3, request and one output");
      const auto tick = decodeTick(readDoubles(prhs[1], 3));
      const double request = readDoubles(prhs[2], 1)[0];
      if (request != 0 && request != 1) throw std::runtime_error("MPC request must be zero or one");
      plhs[0] = encodeTick(runtime().updateMpc(tick, request != 0));
    } else if (command == "query_policy") {
      if (nrhs != 2 || nlhs != 1) throw std::runtime_error("query_policy requires tick3 and one output");
      plhs[0] = encodePolicy(runtime().queryPrepared(decodeTick(readDoubles(prhs[1], 3))));
    } else if (command == "inverse_dynamics") {
      if (nrhs != 2 || nlhs != 1)
        throw std::runtime_error(
            "inverse_dynamics requires queried 95-value policy and one output");
      const double *values = readDoubles(prhs[1], 95);
      const auto torque = runtime().inverseDynamics(decodePolicy(values));
      plhs[0] = mxCreateDoubleMatrix(23, 1, mxREAL);
      std::copy(torque.begin(), torque.end(), mxGetPr(plhs[0]));
    } else {
      throw std::runtime_error("Unknown MEX command");
    }
  } catch (const std::exception &error) {
    mexErrMsgIdAndTxt("G1:PlannerMex", "%s", error.what());
  }
}
