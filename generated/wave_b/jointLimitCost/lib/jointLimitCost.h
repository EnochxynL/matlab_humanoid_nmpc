//
// File: jointLimitCost.h
//
// MATLAB Coder version            : 25.2
// C/C++ source code generated on  : 08-Oct-2026 10:54:53
//

#ifndef JOINTLIMITCOST_H
#define JOINTLIMITCOST_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void jointLimitCost(const double q[23], const double lower[23],
                           const double upper[23], const double config[2],
                           double *b_value, double gradient[58],
                           double hessian[3364]);

#endif
//
// File trailer for jointLimitCost.h
//
// [EOF]
//
