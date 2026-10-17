#include "mex.h"

// Minimal compiler/load probe; independent of controller and Eigen.
void mexFunction(int outputs, mxArray** result, int, const mxArray**) {
  if (outputs > 0) result[0] = mxCreateDoubleScalar(81.0);
}
