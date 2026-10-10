//
// File: nominalState.cpp
//
// MATLAB Coder version            : 25.2
// C/C++ source code generated on  : 08-Oct-2026 11:14:21
//

// Include Files
#include "nominalState.h"
#include "mod.h"
#include "rt_nonfinite.h"
#include "math.h"
#include <algorithm>

// Function Definitions
//
// Raw target58, yaw/phase scalar double, four uint32 indices, logical active.
//  Indices are original shoulder_y left/right, elbow_y left/right in q23.
//
// Arguments    : const double raw[58]
//                double currentYaw
//                double phase
//                const unsigned int jointIndices[4]
//                boolean_T armActive
//                double x[58]
// Return Type  : void
//
void nominalState(const double raw[58], double currentYaw, double phase,
                  const unsigned int jointIndices[4], boolean_T armActive,
                  double x[58])
{
  std::copy(&raw[0], &raw[58], &x[0]);
  if (armActive) {
    double c;
    double s;
    double wave;
    c = cos(currentYaw);
    s = sin(currentYaw);
    wave = sin(6.2831853071795862 * (phase - 0.15));
    c = wave * (c * raw[29] + s * raw[30]);
    if (coder::b_mod(1.0) == 1.0) {
      x[static_cast<int>(jointIndices[0]) + 6] =
          raw[static_cast<int>(jointIndices[0]) + 6] + -0.15 * c;
    } else {
      x[static_cast<int>(jointIndices[0]) + 6] =
          raw[static_cast<int>(jointIndices[0]) + 6] + 0.15 * c;
    }
    if (coder::b_mod(2.0) == 1.0) {
      x[static_cast<int>(jointIndices[1]) + 6] += -0.15 * c;
    } else {
      x[static_cast<int>(jointIndices[1]) + 6] += 0.15 * c;
    }
    if (coder::b_mod(3.0) == 1.0) {
      x[static_cast<int>(jointIndices[2]) + 6] += -0.15 * c;
    } else {
      x[static_cast<int>(jointIndices[2]) + 6] += 0.15 * c;
    }
    if (coder::b_mod(4.0) == 1.0) {
      x[static_cast<int>(jointIndices[3]) + 6] += -0.15 * c;
    } else {
      x[static_cast<int>(jointIndices[3]) + 6] += 0.15 * c;
    }
  }
}

//
// File trailer for nominalState.cpp
//
// [EOF]
//
