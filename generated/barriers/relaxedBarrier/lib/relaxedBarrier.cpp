//
// File: relaxedBarrier.cpp
//
// MATLAB Coder version            : 25.2
// C/C++ source code generated on  : 08-Oct-2026 10:23:19
//

// Include Files
#include "relaxedBarrier.h"
#include <cmath>

// Function Definitions
//
// Original RelaxedBarrierPenalty; time unused. Positive delta, mu>=0.
//
// Arguments    : double margin
//                const double config[2]
//                double *b_value
//                double *first
//                double *second
// Return Type  : void
//
void relaxedBarrier(double margin, const double config[2], double *b_value,
                    double *first, double *second)
{
  if (margin > config[1]) {
    *b_value = -config[0] * std::log(margin);
    *first = -config[0] / margin;
    *second = config[0] / (margin * margin);
  } else {
    double deltaH;
    double deltaH_tmp;
    deltaH_tmp = margin - 2.0 * config[1];
    deltaH = deltaH_tmp / config[1];
    *b_value =
        config[0] * ((-std::log(config[1]) + 0.5 * deltaH * deltaH) - 0.5);
    deltaH = config[1] * config[1];
    *first = config[0] * (deltaH_tmp / deltaH);
    *second = config[0] / deltaH;
  }
}

//
// File trailer for relaxedBarrier.cpp
//
// [EOF]
//
