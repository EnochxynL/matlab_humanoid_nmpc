//
// File: nominalState.h
//
// MATLAB Coder version            : 25.2
// C/C++ source code generated on  : 08-Oct-2026 11:14:21
//

#ifndef NOMINALSTATE_H
#define NOMINALSTATE_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void nominalState(const double raw[58], double currentYaw, double phase,
                         const unsigned int jointIndices[4],
                         boolean_T armActive, double x[58]);

#endif
//
// File trailer for nominalState.h
//
// [EOF]
//
