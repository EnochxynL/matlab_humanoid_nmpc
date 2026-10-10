//
// File: mod.cpp
//
// MATLAB Coder version            : 25.2
// C/C++ source code generated on  : 08-Oct-2026 11:14:21
//

// Include Files
#include "mod.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
//
// Arguments    : double x
// Return Type  : double
//
namespace coder {
double b_mod(double x)
{
  double r;
  if (std::isnan(x) || std::isinf(x)) {
    r = rtNaN;
  } else {
    r = std::fmod(x, 2.0);
    if (r == 0.0) {
      r = 0.0;
    } else if (r < 0.0) {
      r += 2.0;
    }
  }
  return r;
}

} // namespace coder

//
// File trailer for mod.cpp
//
// [EOF]
//
