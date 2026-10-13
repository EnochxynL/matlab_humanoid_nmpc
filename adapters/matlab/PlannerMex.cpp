#include "g1_controller/G1Controller.h"
#include "mex.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

namespace {
using g1_controller::G1Controller;
std::unique_ptr<G1Controller> controller;

std::string readString(const mxArray *argument) {
  if (!mxIsChar(argument))
    throw std::runtime_error("Expected character vector");
  char *text = mxArrayToString(argument);
  std::string result(text);
  mxFree(text);
  return result;
}

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

G1Controller &runtime() {
  if (!controller)
    throw std::runtime_error("Initialize MEX first");
  return *controller;
}

void closeController() {
  if (controller)
    controller->shutdown();
  controller.reset();
}

// These offsets are the unchanged MATLAB transport protocol. No robot math
// or solver scheduling belongs in this adapter.
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

g1_controller::Policy decodePolicy(const double *a) {
  g1_controller::Policy p;
  std::copy_n(a, 58, p.state.begin());
  std::copy_n(a + 58, 35, p.input.begin());
  p.mode = a[93];
  p.queryTime = a[94];
  return p;
}

mxArray *encodePolicy(const g1_controller::Policy &p) {
  mxArray *output = mxCreateDoubleMatrix(95, 1, mxREAL);
  double *a = mxGetPr(output);
  std::copy(p.state.begin(), p.state.end(), a);
  std::copy(p.input.begin(), p.input.end(), a + 58);
  a[93] = p.mode;
  a[94] = p.queryTime;
  return output;
}

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
} // namespace

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
