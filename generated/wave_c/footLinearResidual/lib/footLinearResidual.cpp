//
// File: footLinearResidual.cpp
//
// MATLAB Coder version            : 25.2
// C/C++ source code generated on  : 08-Oct-2026 12:33:00
//

// Include Files
#include "footLinearResidual.h"
#include <algorithm>
#include <cstring>

// Function Definitions
//
// Strict original end-effector dependency supplies f/dx/du, packed 6x94.
//  Original consumers use diagonal matrices (stance) or one z selector (swing).
//  Preserve contribution order: position, velocity, acceleration. No GN change.
//
// Arguments    : const double b[6]
//                const double Ax[36]
//                const double Av[36]
//                const double Aa[36]
//                const boolean_T enabled[3]
//                const double position[564]
//                const double velocity[564]
//                const double acceleration[564]
//                double result[564]
// Return Type  : void
//
void footLinearResidual(const double b[6], const double Ax[36],
                        const double Av[36], const double Aa[36],
                        const boolean_T enabled[3], const double position[564],
                        const double velocity[564],
                        const double acceleration[564], double result[564])
{
  double data[564];
  double A[36];
  std::memset(&result[0], 0, 564U * sizeof(double));
  for (int inner{0}; inner < 6; inner++) {
    result[inner] = b[inner];
  }
  for (int k{0}; k < 3; k++) {
    if (enabled[k]) {
      if (k == 0) {
        std::copy(&Ax[0], &Ax[36], &A[0]);
        std::copy(&position[0], &position[564], &data[0]);
      } else if (k + 1 == 2) {
        std::copy(&Av[0], &Av[36], &A[0]);
        std::copy(&velocity[0], &velocity[564], &data[0]);
      } else {
        std::copy(&Aa[0], &Aa[36], &A[0]);
        std::copy(&acceleration[0], &acceleration[564], &data[0]);
      }
      for (int column{0}; column < 94; column++) {
        for (int row{0}; row < 6; row++) {
          double product;
          product = 0.0;
          for (int inner{0}; inner < 6; inner++) {
            product += A[row + 6 * inner] * data[inner + 6 * column];
          }
          int result_tmp;
          result_tmp = row + 6 * column;
          result[result_tmp] += product;
        }
      }
    }
  }
}

//
// File trailer for footLinearResidual.cpp
//
// [EOF]
//
