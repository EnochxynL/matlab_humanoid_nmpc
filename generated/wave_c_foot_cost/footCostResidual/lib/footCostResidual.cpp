//
// File: footCostResidual.cpp
//
// MATLAB Coder version            : 25.2
// C/C++ source code generated on  : 08-Oct-2026 17:14:34
//

// Include Files
#include "footCostResidual.h"

// Function Definitions
//
// Prototype for U18. geometry: [unused3;planeError3;v3;omega3;a3;alpha3].
//  Parameters retain original 18 reference, 18 sqrt-weight, impact-scaler
//  layout. No position tracking term is invented; the original first three
//  errors are zero.
//
// Arguments    : const double geometry[18]
//                const double parameters[37]
//                double out[18]
// Return Type  : void
//
void footCostResidual(const double geometry[18], const double parameters[37],
                      double out[18])
{
  for (int k{0}; k < 18; k++) {
    double errorValue;
    if (k + 1 <= 3) {
      errorValue = 0.0;
    } else if (k + 1 <= 6) {
      errorValue = geometry[k];
    } else {
      errorValue = geometry[k] - parameters[k];
    }
    out[k] = errorValue * parameters[k + 18] * parameters[36];
  }
}

//
// File trailer for footCostResidual.cpp
//
// [EOF]
//
