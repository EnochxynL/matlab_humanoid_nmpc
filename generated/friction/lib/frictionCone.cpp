//
// File: frictionCone.cpp
//
// MATLAB Coder version            : 25.2
// C/C++ source code generated on  : 08-Oct-2026 08:51:21
//

// Include Files
#include "frictionCone.h"
#include "frictionCone_types.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Original FrictionForceConeConstraint with fixed identity world->terrain.
//  config double4x1: friction coefficient, regularization, gripper force,
//  Hessian diagonal shift (original default1e-6, NOT a tuning change).
//
// Arguments    : const double input[35]
//                unsigned char contactIndex
//                const boolean_T stanceFlags[2]
//                boolean_T enabled
//                const double config[4]
//                boolean_T *active
//                double *b_value
//                struct0_T *a
// Return Type  : void
//
void frictionCone(const double input[35], unsigned char contactIndex,
                  const boolean_T stanceFlags[2], boolean_T enabled,
                  const double config[4], boolean_T *active, double *b_value,
                  struct0_T *a)
{
  double fx2;
  double fy2;
  double fy2_tmp;
  double n;
  double s;
  int first;
  int i;
  int i1;
  if (enabled && stanceFlags[contactIndex]) {
    *active = true;
  } else {
    *active = false;
  }
  first = 6 * contactIndex;
  fx2 = input[first] * input[first];
  fy2_tmp = input[first + 1];
  fy2 = fy2_tmp * fy2_tmp;
  s = (fx2 + fy2) + config[1];
  n = std::sqrt(s);
  s *= n;
  *b_value = config[0] * (input[first + 2] + config[2]) - n;
  std::memset(&a->dfdx[0], 0, 58U * sizeof(double));
  std::memset(&a->dfdu[0], 0, 35U * sizeof(double));
  std::memset(&a->dfdxx[0], 0, 3364U * sizeof(double));
  std::memset(&a->dfduu[0], 0, 1225U * sizeof(double));
  std::memset(&a->dfdux[0], 0, 2030U * sizeof(double));
  a->dfdu[first] = -input[first] / n;
  a->dfdu[first + 1] = -fy2_tmp / n;
  a->dfdu[first + 2] = config[0];
  i = first + 35 * first;
  a->dfduu[i] = -(fy2 + config[1]) / s;
  i1 = first + 35 * (first + 1);
  a->dfduu[i1 + 1] = -(fx2 + config[1]) / s;
  a->dfduu[i1] = input[first] * fy2_tmp / s;
  a->dfduu[i + 1] = a->dfduu[i1];
  for (int j{0}; j < 35; j++) {
    first = j + 35 * j;
    a->dfduu[first] -= config[3];
  }
  for (int j{0}; j < 58; j++) {
    first = j + 58 * j;
    a->dfdxx[first] -= config[3];
  }
}

//
// File trailer for frictionCone.cpp
//
// [EOF]
//
