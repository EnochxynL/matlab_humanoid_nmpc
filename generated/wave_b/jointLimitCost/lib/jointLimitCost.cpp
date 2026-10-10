//
// File: jointLimitCost.cpp
//
// MATLAB Coder version            : 25.2
// C/C++ source code generated on  : 08-Oct-2026 10:54:53
//

// Include Files
#include "jointLimitCost.h"
#include "polynomialBarrier.h"
#include <cstring>

// Function Definitions
//
// q/lower/upper real double23x1; config double2x1 [mu;delta].
//  The WB model packs actuated q at x(7:29); offset in the original is zero.
//
// Arguments    : const double q[23]
//                const double lower[23]
//                const double upper[23]
//                const double config[2]
//                double *b_value
//                double gradient[58]
//                double hessian[3364]
// Return Type  : void
//
void jointLimitCost(const double q[23], const double lower[23],
                    const double upper[23], const double config[2],
                    double *b_value, double gradient[58], double hessian[3364])
{
  double lo[23];
  double up[23];
  double dl;
  double du;
  double hl;
  double hu;
  double y;
  std::memset(&gradient[0], 0, 58U * sizeof(double));
  std::memset(&hessian[0], 0, 3364U * sizeof(double));
  for (int k{0}; k < 23; k++) {
    y = q[k];
    up[k] = polynomialBarrier(upper[k] - y, config, du, hu);
    lo[k] = polynomialBarrier(y - lower[k], config, dl, hl);
    gradient[k + 6] = dl - du;
    hessian[(k + 58 * (k + 6)) + 6] = hl + hu;
  }
  y = up[0];
  du = lo[0];
  for (int k{0}; k < 22; k++) {
    y += up[k + 1];
    du += lo[k + 1];
  }
  *b_value = y + du;
}

//
// File trailer for jointLimitCost.cpp
//
// [EOF]
//
