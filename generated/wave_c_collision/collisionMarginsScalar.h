#pragma once
#include <cmath>
namespace task5c::collision {
template<typename Scalar>
void collisionMarginsScalar(const Scalar positions[30], const Scalar radii[2],
                      Scalar out[16])
{
  using std::sqrt;
  static const signed char pairs[32]{5, 5, 7, 7,  3, 3, 4, 4, 3, 9, 3,
                                     5, 7, 4, 6,  8, 6, 8, 6, 8, 6, 8,
                                     5, 7, 4, 10, 2, 2, 2, 1, 1, 1};
  Scalar footDistance;
  Scalar kneeDistance;
  footDistance = 2.0 * radii[0];
  kneeDistance = 2.0 * radii[1];
  for (int k{0}; k < 16; k++) {
    Scalar d_idx_0;
    Scalar d_idx_1;
    Scalar d_idx_2;
    int b_d_idx_0_tmp;
    int d_idx_0_tmp;
    d_idx_0_tmp = 3 * (pairs[k] - 1);
    b_d_idx_0_tmp = 3 * (pairs[k + 16] - 1);
    d_idx_0 = positions[d_idx_0_tmp] - positions[b_d_idx_0_tmp];
    d_idx_1 = positions[d_idx_0_tmp + 1] - positions[b_d_idx_0_tmp + 1];
    d_idx_2 = positions[d_idx_0_tmp + 2] - positions[b_d_idx_0_tmp + 2];
    //  Eigen fixed vector3 sum reduction: coefficient 0 + (coefficient 1 + 2).
    d_idx_0 =
        sqrt(d_idx_0 * d_idx_0 + (d_idx_1 * d_idx_1 + d_idx_2 * d_idx_2));
    if (k + 1 == 10) {
      out[9] = d_idx_0 - kneeDistance;
    } else {
      out[k] = d_idx_0 - footDistance;
    }
  }
}
}
