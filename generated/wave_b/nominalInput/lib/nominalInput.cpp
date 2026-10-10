//
// File: nominalInput.cpp
//
// MATLAB Coder version            : 25.2
// C/C++ source code generated on  : 08-Oct-2026 11:14:25
//

// Include Files
#include "nominalInput.h"
#include <cstring>

// Function Definitions
//
// Original model mass is an immutable Pinocchio dependency; world wrench35.
//
// Arguments    : double mass
//                const boolean_T contactFlags[2]
//                double u[35]
// Return Type  : void
//
void nominalInput(double mass, const boolean_T contactFlags[2], double u[35])
{
  double force;
  int count;
  force = mass * 9.81;
  count = contactFlags[0] + contactFlags[1];
  std::memset(&u[0], 0, 35U * sizeof(double));
  if (count > 0) {
    if (contactFlags[0]) {
      u[2] = force / static_cast<double>(count);
    }
    if (contactFlags[1]) {
      u[8] = force / static_cast<double>(count);
    }
  }
}

//
// File trailer for nominalInput.cpp
//
// [EOF]
//
