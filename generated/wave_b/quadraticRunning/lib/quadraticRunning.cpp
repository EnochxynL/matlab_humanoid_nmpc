//
// File: quadraticRunning.cpp
//
// MATLAB Coder version            : 25.2
// C/C++ source code generated on  : 08-Oct-2026 11:01:35
//

// Include Files
#include "quadraticRunning.h"
#include "quadraticRunning_types.h"
#include <algorithm>
#include <cstring>
#include <emmintrin.h>

// Function Definitions
//
// Frozen original WB P is empty. Preserve .5 convention and all derivatives.
//
// Arguments    : const double x[58]
//                const double u[35]
//                const double nominalX[58]
//                const double nominalU[35]
//                const double q[3364]
//                const double r[1225]
//                Task5CQuadraticRunning *approximation
// Return Type  : void
//
void quadraticRunning(const double x[58], const double u[35],
                      const double nominalX[58], const double nominalU[35],
                      const double q[3364], const double r[1225],
                      Task5CQuadraticRunning *approximation)
{
  __m128d b_r;
  double dx[58];
  double du[35];
  double b_du;
  double b_dx;
  for (int i{0}; i <= 56; i += 2) {
    _mm_storeu_pd(&dx[i],
                  _mm_sub_pd(_mm_loadu_pd(&x[i]), _mm_loadu_pd(&nominalX[i])));
  }
  for (int i{0}; i <= 32; i += 2) {
    _mm_storeu_pd(&du[i],
                  _mm_sub_pd(_mm_loadu_pd(&u[i]), _mm_loadu_pd(&nominalU[i])));
  }
  du[34] = u[34] - nominalU[34];
  std::copy(&q[0], &q[3364], &approximation->dfdxx[0]);
  std::copy(&r[0], &r[1225], &approximation->dfduu[0]);
  std::memset(&approximation->dfdux[0], 0, 2030U * sizeof(double));
  std::memset(&approximation->dfdx[0], 0, 58U * sizeof(double));
  for (int i{0}; i < 58; i++) {
    for (int b_i{0}; b_i <= 56; b_i += 2) {
      b_r = _mm_loadu_pd(&approximation->dfdx[b_i]);
      _mm_storeu_pd(&approximation->dfdx[b_i],
                    _mm_add_pd(b_r, _mm_mul_pd(_mm_loadu_pd(&q[b_i + 58 * i]),
                                               _mm_set1_pd(dx[i]))));
    }
  }
  std::memset(&approximation->dfdu[0], 0, 35U * sizeof(double));
  for (int i{0}; i < 35; i++) {
    for (int b_i{0}; b_i <= 32; b_i += 2) {
      b_r = _mm_loadu_pd(&approximation->dfdu[b_i]);
      _mm_storeu_pd(&approximation->dfdu[b_i],
                    _mm_add_pd(b_r, _mm_mul_pd(_mm_loadu_pd(&r[b_i + 35 * i]),
                                               _mm_set1_pd(du[i]))));
    }
    approximation->dfdu[34] += r[35 * i + 34] * du[i];
  }
  b_dx = 0.0;
  for (int i{0}; i < 58; i++) {
    b_dx += dx[i] * approximation->dfdx[i];
  }
  b_du = 0.0;
  for (int i{0}; i < 35; i++) {
    b_du += du[i] * approximation->dfdu[i];
  }
  approximation->f = 0.5 * b_dx + 0.5 * b_du;
}

//
// File trailer for quadraticRunning.cpp
//
// [EOF]
//
