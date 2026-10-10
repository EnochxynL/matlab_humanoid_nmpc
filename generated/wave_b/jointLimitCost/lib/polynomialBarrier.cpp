//
// File: polynomialBarrier.cpp
//
// MATLAB Coder version            : 25.2
// C/C++ source code generated on  : 08-Oct-2026 10:54:53
//

// Include Files
#include "polynomialBarrier.h"

// Function Definitions
//
// Config double2x1 [mu;delta]; real scalar margin. Preserve executed source,
//  not the inconsistent formula printed in the upstream header comment.
//
// Arguments    : double margin
//                const double config[2]
//                double &first
//                double &second
// Return Type  : double
//
double polynomialBarrier(double margin, const double config[2], double &first,
                         double &second)
{
  double b_value;
  if (margin <= 0.0) {
    b_value = ((0.5 * margin * margin - config[1] * margin / 2.0) +
               config[1] * config[1] / 6.0) *
              config[0];
    first = (margin - config[1] / 2.0) * config[0];
    second = config[0];
  } else if ((margin > 0.0) && (margin < config[1])) {
    double value_tmp;
    value_tmp = -margin * margin;
    b_value =
        (((value_tmp * margin / (6.0 * config[1]) + 0.5 * margin * margin) -
          config[1] * margin / 2.0) +
         config[1] * config[1] / 6.0) *
        config[0];
    first = ((value_tmp / (2.0 * config[1]) + margin) - config[1] / 2.0) *
            config[0];
    second = config[0] * (-margin / config[1] + 1.0);
  } else {
    b_value = 0.0;
    first = 0.0;
    second = 0.0;
  }
  return b_value;
}

//
// File trailer for polynomialBarrier.cpp
//
// [EOF]
//
