//
// File: quadraticTerminal.cpp
//
// MATLAB Coder version            : 25.2
// C/C++ source code generated on  : 08-Oct-2026 11:01:30
//

// Include Files
#include "quadraticTerminal.h"
#include <algorithm>
#include <cstring>
#include <emmintrin.h>

// Function Definitions
//
// x/desired real double58x1; q real double58x58 (original scaled Q_final).
//
// Arguments    : const double x[58]
//                const double desired[58]
//                const double q[3364]
//                double *b_value
//                double gradient[58]
//                double hessian[3364]
// Return Type  : void
//
void quadraticTerminal(const double x[58], const double desired[58],
                       const double q[3364], double *b_value,
                       double gradient[58], double hessian[3364])
{
  double deviation[58];
  double b_deviation;
  for (int i{0}; i <= 56; i += 2) {
    _mm_storeu_pd(&deviation[i],
                  _mm_sub_pd(_mm_loadu_pd(&x[i]), _mm_loadu_pd(&desired[i])));
  }
  std::copy(&q[0], &q[3364], &hessian[0]);
  std::memset(&gradient[0], 0, 58U * sizeof(double));
  for (int i{0}; i < 58; i++) {
    for (int b_i{0}; b_i <= 56; b_i += 2) {
      __m128d r;
      r = _mm_loadu_pd(&gradient[b_i]);
      _mm_storeu_pd(&gradient[b_i],
                    _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&q[b_i + 58 * i]),
                                             _mm_set1_pd(deviation[i]))));
    }
  }
  b_deviation = 0.0;
  for (int i{0}; i < 58; i++) {
    b_deviation += deviation[i] * gradient[i];
  }
  *b_value = 0.5 * b_deviation;
}

//
// File trailer for quadraticTerminal.cpp
//
// [EOF]
//
