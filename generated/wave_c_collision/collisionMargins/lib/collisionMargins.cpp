//
// File: collisionMargins.cpp
//
// MATLAB Coder version            : 25.2
// C/C++ source code generated on  : 08-Oct-2026 16:23:07
//

// Include Files
#include "collisionMargins.h"
#include <cmath>

// Function Definitions
//
// Exact original point order: ankle L/R, center L/R, p1 L/R, p2 L/R, knee L/R.
//  Original runtime uses Euclidean norm, despite a squared-distance comment.
//
// Arguments    : const double positions[30]
//                const double radii[2]
//                double out[16]
// Return Type  : void
//
void collisionMargins(const double positions[30], const double radii[2],
                      double out[16])
{
  static const signed char pairs[32]{5, 5, 7, 7,  3, 3, 4, 4, 3, 9, 3,
                                     5, 7, 4, 6,  8, 6, 8, 6, 8, 6, 8,
                                     5, 7, 4, 10, 2, 2, 2, 1, 1, 1};
  double footDistance;
  double kneeDistance;
  footDistance = 2.0 * radii[0];
  kneeDistance = 2.0 * radii[1];
  for (int k{0}; k < 16; k++) {
    double d_idx_0;
    double d_idx_1;
    double d_idx_2;
    int b_d_idx_0_tmp;
    int d_idx_0_tmp;
    d_idx_0_tmp = 3 * (pairs[k] - 1);
    b_d_idx_0_tmp = 3 * (pairs[k + 16] - 1);
    d_idx_0 = positions[d_idx_0_tmp] - positions[b_d_idx_0_tmp];
    d_idx_1 = positions[d_idx_0_tmp + 1] - positions[b_d_idx_0_tmp + 1];
    d_idx_2 = positions[d_idx_0_tmp + 2] - positions[b_d_idx_0_tmp + 2];
    //  Eigen fixed vector3 sum reduction: coefficient 0 + (coefficient 1 + 2).
    d_idx_0 =
        std::sqrt(d_idx_0 * d_idx_0 + (d_idx_1 * d_idx_1 + d_idx_2 * d_idx_2));
    if (k + 1 == 10) {
      out[9] = d_idx_0 - kneeDistance;
    } else {
      out[k] = d_idx_0 - footDistance;
    }
  }
}

//
// File trailer for collisionMargins.cpp
//
// [EOF]
//
