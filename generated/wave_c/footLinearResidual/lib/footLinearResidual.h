//
// File: footLinearResidual.h
//
// MATLAB Coder version            : 25.2
// C/C++ source code generated on  : 08-Oct-2026 12:33:00
//

#ifndef FOOTLINEARRESIDUAL_H
#define FOOTLINEARRESIDUAL_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void
footLinearResidual(const double b[6], const double Ax[36], const double Av[36],
                   const double Aa[36], const boolean_T enabled[3],
                   const double position[564], const double velocity[564],
                   const double acceleration[564], double result[564]);

#endif
//
// File trailer for footLinearResidual.h
//
// [EOF]
//
