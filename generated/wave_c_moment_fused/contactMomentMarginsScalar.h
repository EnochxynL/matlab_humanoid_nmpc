// Mechanical scalar-type lifting of MATLAB Coder output; algebra unchanged.
#pragma once
namespace task5c::momentfused {
template<typename Scalar>
void contactMomentMarginsScalar(const Scalar bounds[4], const Scalar localWrench[564],
                          Scalar out[376])
{
  Scalar d;
  Scalar d1;
  Scalar d2;
  Scalar d3;
  d = bounds[2];
  d1 = bounds[3];
  d2 = bounds[0];
  d3 = bounds[1];
  for (int k{0}; k < 94; k++) {
    Scalar d4;
    Scalar d5;
    int out_tmp;
    d4 = localWrench[6 * k + 3];
    d5 = localWrench[6 * k + 2];
    out_tmp = k << 2;
    out[out_tmp] = d4 - d * d5;
    out[out_tmp + 1] = -d4 + d1 * d5;
    d4 = localWrench[6 * k + 4];
    out[out_tmp + 2] = -d4 - d2 * d5;
    out[out_tmp + 3] = d4 + d3 * d5;
  }
}
}
