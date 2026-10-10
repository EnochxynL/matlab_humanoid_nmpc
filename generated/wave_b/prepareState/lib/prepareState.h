//
// File: prepareState.h
//
// MATLAB Coder version            : 25.2
// C/C++ source code generated on  : 08-Oct-2026 11:14:16
//

#ifndef PREPARESTATE_H
#define PREPARESTATE_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void prepareState(const double position[3], const double quaternion[4],
                         const double q[23], const double linearLocal[3],
                         const double angularLocal[3], const double qd[23],
                         double state[58]);

#endif
//
// File trailer for prepareState.h
//
// [EOF]
//
