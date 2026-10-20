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
  if (!mxIsDouble(argument) || mxIsComplex(argument) || mxIsSparse(argument) ||
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

// Node widths follow the selected OCP. The whole-body layout remains unchanged;
// centroidal uses the same bounded packet structure with 35-state nodes.


/**
 * @brief 解码参考包（全身 428 项、centroidal 359 项）；publish 为假时直接返回。
 * 包含 tick、至多 3 个目标节点、至多 64 个事件和 65 个模式，
 * 以及终止时间、摆腿参数和滤波命令；实际计数由核心继续校验。
 */
g1_controller::Reference decodeReference(const double *a, const g1_controller::ProblemDimensions& dimensions) {
  g1_controller::Reference r;
  if (a[1] != 0 && a[1] != 1) throw std::runtime_error("Invalid reference publication flag");
  r.publish = a[1] != 0;
  if (!r.publish)
    return r;
  if (a[0] < 0 || a[0] > 9007199254740992.0 || std::floor(a[0]) != a[0])
    throw std::runtime_error("Invalid reference tick");
  if (a[2] != 1 && a[2] != 3) throw std::runtime_error("Invalid reference node count");
  r.tick = static_cast<std::uint64_t>(a[0]);
  r.targetCount = static_cast<std::size_t>(a[2]);
  std::copy_n(a + 3, 3, r.targetTimes.begin());
  for (std::size_t k = 0; k < 3; ++k) {
    r.targetStates[k].resize(dimensions.state);
    r.targetInputs[k].resize(dimensions.input);
    std::copy_n(a + 6 + dimensions.state * k, dimensions.state, r.targetStates[k].begin());
    std::copy_n(a + 6 + 3*dimensions.state + dimensions.input * k, dimensions.input, r.targetInputs[k].begin());
  }
  const auto offset = 6 + 3*(dimensions.state+dimensions.input);
  if (a[offset] < 1 || a[offset] > 64 || std::floor(a[offset]) != a[offset])
    throw std::runtime_error("Invalid reference event count");
  r.eventCount = static_cast<std::size_t>(a[offset]);
  std::copy_n(a + offset+1, 64, r.eventTimes.begin());
  for (std::size_t k = 0; k < 65; ++k) {
    if (a[offset+65+k] < 0 || a[offset+65+k] > 3 || std::floor(a[offset+65+k]) != a[offset+65+k])
      throw std::runtime_error("Invalid reference contact mode");
    r.modes[k] = static_cast<std::size_t>(a[offset+65 + k]);
  }
  r.terminalTime = a[offset+130];
  std::copy_n(a + offset+131, 8, r.swingParameters.begin());
  std::copy_n(a + offset+139, 4, r.filteredCommand.begin());
  return r;
}


/**
 * @brief 创建 [state; input; mode; queryTime] 列向量：全身 95 项，centroidal 72 项。
 */
mxArray *encodePolicy(const g1_controller::Policy &p) {
  const auto stateSize = p.state.size(), inputSize = p.input.size();
  mxArray *output = mxCreateDoubleMatrix(stateSize+inputSize+2, 1, mxREAL);
  double *a = mxGetPr(output);
  std::copy(p.state.begin(), p.state.end(), a);
  std::copy(p.input.begin(), p.input.end(), a + stateSize);
  a[stateSize+inputSize] = p.mode;
  a[stateSize+inputSize+1] = p.queryTime;
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

mxArray* encodeDiagnostics(const g1_controller::DiagnosticBatch& batch) {
  const char* fields[]={"tables","snapshot","configurationJson","loadedLibraries"};
  auto* result=mxCreateStructMatrix(1,1,4,fields);
  const char* tableFields[]={"name","columns","values"};
  auto* tables=mxCreateStructMatrix(1,batch.tables.size(),3,tableFields);
  for(size_t k=0;k<batch.tables.size();++k) {
    const auto& table=batch.tables[k];
    mxSetField(tables,k,"name",mxCreateString(table.name.c_str()));
    auto* columns=mxCreateCellMatrix(1,table.columns.size());
    for(size_t j=0;j<table.columns.size();++j)mxSetCell(columns,j,mxCreateString(table.columns[j].c_str()));
    mxSetField(tables,k,"columns",columns);
    auto* values=mxCreateCellMatrix(table.rows.size(),table.columns.size());
    for(size_t i=0;i<table.rows.size();++i)for(size_t j=0;j<table.columns.size();++j) {
      const auto& value=table.rows[i][j];
      mxSetCell(values,i+j*table.rows.size(),value.textual?mxCreateString(value.text.c_str()):mxCreateDoubleScalar(value.number));
    }
    mxSetField(tables,k,"values",values);
  }
  mxSetField(result,0,"tables",tables);
  auto* snapshot=mxCreateDoubleMatrix(batch.snapshot.size(),1,mxREAL);
  std::copy(batch.snapshot.begin(),batch.snapshot.end(),mxGetPr(snapshot));
  mxSetField(result,0,"snapshot",snapshot);
  mxSetField(result,0,"configurationJson",mxCreateString(batch.configurationJson.c_str()));
  mxSetField(result,0,"loadedLibraries",mxCreateString(batch.loadedLibraries.c_str()));
  return result;
}
} // namespace

/**
 * @brief MEX 命令分发入口，仅暴露当前分阶段运行接口。
 * init(config)：使用 MATLAB 提供的路径创建核心并 mexLock；close：关闭并解锁。
 * dimensions → [state;input;joints]；convert_observation(qv) → 所选问题的状态。
 * install_reference(packet)：安装已提取参考。
 * install_observation(state, identity2) → tick3：登记独立提取的观测，不求解。
 * update_mpc(tick3,request) → tick3：按外部请求同步求解或沿用策略。
 * query_policy(tick3) → policy：只查询已准备好的策略，不触发求解。
 * 动作计算和完成检查由 MATLAB 协调层负责，力矩不回传本 MEX。
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
      g1_controller::ControllerConfig config{field("taskFile"), field("referenceFile"),
                                                  field("urdfFile"), field("logDirectory")};
      if (const auto* selected = mxGetField(prhs[1], 0, "formulation")) {
        const auto name = readString(selected);
        if (name == "centroidal") config.formulation = g1_controller::MpcFormulation::Centroidal;
        else if (name != "whole_body") throw std::runtime_error("Unknown MPC formulation");
      }
      if (const auto* enabled = mxGetField(prhs[1], 0, "diagnosticsEnabled")) {
        if (!mxIsLogicalScalar(enabled)) throw std::runtime_error("diagnosticsEnabled must be a logical scalar");
        config.diagnosticsEnabled = mxIsLogicalScalarTrue(enabled);
      }
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
    } else if (command == "dimensions") {
      if (nrhs!=1 || nlhs!=1) throw std::runtime_error("dimensions requires one output");
      const auto d = runtime().dimensions();
      plhs[0] = mxCreateDoubleMatrix(3,1,mxREAL);
      auto* a = mxGetPr(plhs[0]); a[0]=d.state; a[1]=d.input; a[2]=d.joints;
    } else if (command == "convert_observation") {
      if (nrhs!=2 || nlhs!=1) throw std::runtime_error("convert_observation requires generalized state and one output");
      const auto count = 2*(6+runtime().dimensions().joints);
      const auto* values = readDoubles(prhs[1],count);
      const auto state = runtime().observationFromGeneralizedState({values,values+count});
      plhs[0] = mxCreateDoubleMatrix(state.size(),1,mxREAL);
      std::copy(state.begin(),state.end(),mxGetPr(plhs[0]));
    } else if (command == "diagnostics") {
      if (nrhs!=1 || nlhs!=1) throw std::runtime_error("diagnostics requires one output");
      plhs[0]=encodeDiagnostics(runtime().drainDiagnostics());
    } else if (command == "close") {
      closeController();
      if (mexIsLocked())
        mexUnlock();
    } else if (command == "install_reference") {
      if (nrhs != 2)
        throw std::runtime_error("install_reference requires one packet");
      const auto dimensions = runtime().dimensions();
      const double *values = readDoubles(prhs[1], 149+3*(dimensions.state+dimensions.input));
      runtime().setReference(decodeReference(values,dimensions));
    } else if (command == "install_observation") {
      if(nrhs!=3 || nlhs!=1) throw std::runtime_error("install_observation requires state, identity2 and one output");
      const auto count = runtime().dimensions().state;
      std::vector<double> state(count);
      std::array<double,2> identity;
      std::copy_n(readDoubles(prhs[1],count),count,state.begin());
      std::copy_n(readDoubles(prhs[2],2),2,identity.begin());
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
    } else {
      throw std::runtime_error("Unknown MEX command");
    }
  } catch (const std::exception &error) {
    mexErrMsgIdAndTxt("G1:PlannerMex", "%s", error.what());
  }
}
