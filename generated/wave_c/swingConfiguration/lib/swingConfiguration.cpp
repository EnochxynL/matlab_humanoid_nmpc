//
// File: swingConfiguration.cpp
//
// MATLAB Coder version            : 25.2
// C/C++ source code generated on  : 08-Oct-2026 12:32:56
//

// Include Files
#include "swingConfiguration.h"

// Function Definitions
//
// z=[original planner z,zDot,zDDot]; gains=[position,velocity,acceleration].
//  Original source uses this exact subtraction order, not a dot-product
//  rewrite.
//
// Arguments    : const double z[3]
//                const double gains[3]
//                boolean_T positionEnabled
//                double *b
//                double Ax[3]
//                double Av[3]
//                double Aa[3]
// Return Type  : void
//
void swingConfiguration(const double z[3], const double gains[3],
                        boolean_T positionEnabled, double *b, double Ax[3],
                        double Av[3], double Aa[3])
{
  *b = -gains[1] * z[1] - gains[2] * z[2];
  Ax[0] = 0.0;
  Ax[1] = 0.0;
  Ax[2] = 0.0;
  Av[0] = 0.0;
  Av[1] = 0.0;
  Av[2] = gains[1];
  Aa[0] = 0.0;
  Aa[1] = 0.0;
  Aa[2] = gains[2];
  if (positionEnabled) {
    *b -= gains[0] * z[0];
    Ax[2] = gains[0];
  }
}

//
// File trailer for swingConfiguration.cpp
//
// [EOF]
//
