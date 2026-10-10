//
// File: contactMomentMargins.cpp
//
// MATLAB Coder version            : 25.2
// C/C++ source code generated on  : 08-Oct-2026 16:04:59
//

// Include Files
#include "contactMomentMargins.h"

// Function Definitions
//
// bounds=[xmin;xmax;ymin;ymax]; packed [f, dx58, du35].
//  Original geometry/world-to-local AD is a strict dependency, not migrated.
//
// Arguments    : const double bounds[4]
//                const double localWrench[564]
//                double out[376]
// Return Type  : void
//
void contactMomentMargins(const double bounds[4], const double localWrench[564],
                          double out[376])
{
  double d;
  double d1;
  double d2;
  double d3;
  d = bounds[2];
  d1 = bounds[3];
  d2 = bounds[0];
  d3 = bounds[1];
  for (int k{0}; k < 94; k++) {
    double d4;
    double d5;
    int out_tmp;
    d4 = localWrench[6 * k + 3];
    d5 = localWrench[6 * k + 2];
    out_tmp = k << 2;
    out[out_tmp] = d4 - d * d5;
    out[out_tmp + 1] = -d4 + d1 * d5;
    d4 = localWrench[6 * k + 4];
    out[out_tmp + 2] = -d4 - d2 * d5;
    out[out_tmp + 3] = d4 + d3 * d5;
  }
}

//
// File trailer for contactMomentMargins.cpp
//
// [EOF]
//
