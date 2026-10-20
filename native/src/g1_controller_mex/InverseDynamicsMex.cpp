#include "g1_controller/InverseDynamics.h"
#include "mex.h"
#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

namespace {
std::unique_ptr<g1_controller::InverseDynamics> model;
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
std::array<double, N> readVector(const mxArray* value) {
  if (!mxIsDouble(value) || mxIsComplex(value) || mxIsSparse(value) ||
      mxGetNumberOfElements(value) != N)
    throw std::runtime_error("Invalid numeric argument dimensions");
  std::array<double, N> result;
  std::copy_n(mxGetPr(value), N, result.begin());
  for (double x : result)
    if (!std::isfinite(x)) throw std::runtime_error("Nonfinite argument");
  return result;
}
}

/** 独立模型生命周期；compute(x58,u35) 不访问另一 MEX 的任何原生对象。 */
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
      model = std::make_unique<g1_controller::InverseDynamics>(field("taskFile"), field("urdfFile"));
      mexLock();
    } else if (command == "close") {
      if (nrhs != 1 || nlhs) throw std::runtime_error("close takes no arguments or outputs");
      closeModel();
      if (mexIsLocked()) mexUnlock();
    } else if (command == "compute") {
      if (nrhs != 3 || nlhs != 1) throw std::runtime_error("compute requires x58, u35 and one output");
      if (!model) throw std::runtime_error("Initialize inverse dynamics MEX first");
      const auto torque = model->compute(readVector<58>(prhs[1]), readVector<35>(prhs[2]));
      plhs[0] = mxCreateDoubleMatrix(torque.size(), 1, mxREAL);
      std::copy(torque.begin(), torque.end(), mxGetPr(plhs[0]));
    } else throw std::runtime_error("Unknown inverse dynamics command");
  } catch (const std::exception& error) {
    mexErrMsgIdAndTxt("G1:InverseDynamicsMex", "%s", error.what());
  }
}
