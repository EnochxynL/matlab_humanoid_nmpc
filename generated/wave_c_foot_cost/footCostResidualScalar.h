#pragma once
namespace task5c::footcost {
template<typename Scalar>
void footCostResidualScalar(const Scalar geometry[18], const Scalar parameters[37],
                      Scalar out[18])
{
  for (int k{0}; k < 18; k++) {
    Scalar errorValue;
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
}
