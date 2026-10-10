//
// File: zeroWrench.h
//
// MATLAB Coder version            : 25.2
// C/C++ source code generated on  : 08-Oct-2026 08:46:20
//

#ifndef ZEROWRENCH_H
#define ZEROWRENCH_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void zeroWrench(const double input[35], unsigned char contactIndex,
                       const boolean_T stanceFlags[2], boolean_T enabled,
                       boolean_T *active, double b_value[6], double dfdx[348],
                       double dfdu[210]);

#endif
//
// File trailer for zeroWrench.h
//
// [EOF]
//
