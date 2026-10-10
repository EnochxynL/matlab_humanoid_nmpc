//
// File: zeroWrench.cpp
//
// MATLAB Coder version            : 25.2
// C/C++ source code generated on  : 08-Oct-2026 08:46:20
//

// Include Files
#include "zeroWrench.h"
#include <cstring>

// Function Definitions
//
// MATLAB-authored original ZeroWrenchConstraint mathematics.
//  input: real double35x1; contactIndex: uint8 scalar 0/1;
//  stanceFlags: logical2x1 from ORIGINAL reference manager at the exact time.
//  enabled: logical scalar, original setActive state.
//  Value/linear approximation remain defined when inactive (OCS2 gates calls).
//
// Arguments    : const double input[35]
//                unsigned char contactIndex
//                const boolean_T stanceFlags[2]
//                boolean_T enabled
//                boolean_T *active
//                double b_value[6]
//                double dfdx[348]
//                double dfdu[210]
// Return Type  : void
//
void zeroWrench(const double input[35], unsigned char contactIndex,
                const boolean_T stanceFlags[2], boolean_T enabled,
                boolean_T *active, double b_value[6], double dfdx[348],
                double dfdu[210])
{
  int first;
  if (enabled && (!stanceFlags[contactIndex])) {
    *active = true;
  } else {
    *active = false;
  }
  std::memset(&dfdx[0], 0, 348U * sizeof(double));
  std::memset(&dfdu[0], 0, 210U * sizeof(double));
  first = 6 * contactIndex;
  for (int j{0}; j < 6; j++) {
    int value_tmp;
    value_tmp = first + j;
    b_value[j] = input[value_tmp];
    dfdu[j + 6 * value_tmp] = 1.0;
  }
}

//
// File trailer for zeroWrench.cpp
//
// [EOF]
//
