#include <humanoid_common_mpc/pinocchio_model/createPinocchioModel.h>
#include <humanoid_wb_mpc/dynamics/DynamicsHelperFunctions.h>
#include <humanoid_wb_mpc/common/WBAccelMpcRobotModel.h>
#include "mex.h"
#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

namespace {
// MEX 自己持有独立模型；数学实现仍在共享 model 层。
struct InverseDynamicsModel {
  ocs2::humanoid::ModelSettings settings;
  ocs2::PinocchioInterface pinocchio;
  ocs2::humanoid::WBAccelMpcRobotModel<ocs2::scalar_t> robotModel;
  InverseDynamicsModel(const std::string& taskFile,const std::string& urdfFile)
      : settings(taskFile,urdfFile,"wb_mpc_",true),
        pinocchio(ocs2::humanoid::createCustomPinocchioInterface(taskFile,urdfFile,settings)),
        robotModel(settings) {}
};
std::unique_ptr<InverseDynamicsModel> model;
void closeModel() { model.reset(); }
std::string readString(const mxArray* value) {
  if (!mxIsChar(value)) throw std::runtime_error("Expected character vector");
  char* text = mxArrayToString(value);
  if (!text) throw std::runtime_error("Cannot read character vector");
  std::string result(text);
  mxFree(text);
  return result;
}
template<size_t N>
Eigen::Map<const ocs2::vector_t> readVector(const mxArray* value) {
  if (!mxIsDouble(value) || mxIsComplex(value) || mxIsSparse(value) ||
      mxGetNumberOfElements(value) != N)
    throw std::runtime_error("Invalid numeric argument dimensions");
  const auto* data=mxGetPr(value);
  for(size_t k=0;k<N;++k)
    if(!std::isfinite(data[k])) throw std::runtime_error("Nonfinite argument");
  return Eigen::Map<const ocs2::vector_t>(data,N);
}
}

/** 独立模型生命周期；compute(x58,u35) 不访问另一 MEX 的任何原生对象。 */
// Pinocchio 的显式 DLL 导出会关闭 MinGW 的隐式 export-all。
#ifdef _WIN32
__declspec(dllexport)
#endif
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
  try {
    if (!nrhs) throw std::runtime_error("Expected inverse dynamics command");
    const auto command = readString(prhs[0]);
    if (command == "init") {
      if (nrhs != 2 || nlhs || !mxIsStruct(prhs[1]) || mxGetNumberOfElements(prhs[1]) != 1)
        throw std::runtime_error("init requires a scalar configuration and no outputs");
      if (model) throw std::runtime_error("Close existing inverse dynamics model first");
      static bool registered = false;
      if (!registered) { mexAtExit(closeModel); registered = true; }
      const auto field = [&](const char* name) {
        const auto* value = mxGetField(prhs[1], 0, name);
        if (!value) throw std::runtime_error(std::string("Missing configuration field: ") + name);
        return readString(value);
      };
      model = std::make_unique<InverseDynamicsModel>(field("taskFile"), field("urdfFile"));
      mexLock();
    } else if (command == "close") {
      if (nrhs != 1 || nlhs) throw std::runtime_error("close takes no arguments or outputs");
      closeModel();
      if (mexIsLocked()) mexUnlock();
    } else if (command == "compute") {
      if (nrhs != 3 || nlhs != 1) throw std::runtime_error("compute requires x58, u35 and one output");
      if (!model) throw std::runtime_error("Initialize inverse dynamics MEX first");
      const auto state=readVector<58>(prhs[1]);
      const auto input=readVector<35>(prhs[2]);
      const auto torque=ocs2::humanoid::computeJointTorques<ocs2::scalar_t>(
          state,input,model->pinocchio,model->robotModel);
      if(torque.size()!=23 || !torque.allFinite())
        throw std::runtime_error("Invalid inverse dynamics output");
      plhs[0] = mxCreateDoubleMatrix(torque.size(), 1, mxREAL);
      std::copy_n(torque.data(),torque.size(),mxGetPr(plhs[0]));
    } else throw std::runtime_error("Unknown inverse dynamics command");
  } catch (const std::exception& error) {
    mexErrMsgIdAndTxt("G1:InverseDynamicsMex", "%s", error.what());
  }
}
