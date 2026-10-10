//
// File: prepareState.cpp
//
// MATLAB Coder version            : 25.2
// C/C++ source code generated on  : 08-Oct-2026 11:14:16
//

// Include Files
#include "prepareState.h"
#include "math.h"
#include <cmath>
#include <emmintrin.h>

// Function Definitions
//
// Fixed real double inputs:3,4(wxyz),23,3,3,23 column vectors.
//
// Arguments    : const double position[3]
//                const double quaternion[4]
//                const double q[23]
//                const double linearLocal[3]
//                const double angularLocal[3]
//                const double qd[23]
//                double state[58]
// Return Type  : void
//
void prepareState(const double position[3], const double quaternion[4],
                  const double q[23], const double linearLocal[3],
                  const double angularLocal[3], const double qd[23],
                  double state[58])
{
  __m128d r;
  double uv[3];
  double cx;
  double cy;
  double d;
  double pitch;
  double roll;
  double sx;
  double sy;
  double yaw;
  cy = quaternion[2] * quaternion[2];
  yaw = atan2(
      2.0 * (quaternion[0] * quaternion[3] + quaternion[1] * quaternion[2]),
      1.0 - 2.0 * (cy + quaternion[3] * quaternion[3]));
  pitch = std::asin(
      2.0 * (quaternion[0] * quaternion[2] - quaternion[1] * quaternion[3]));
  roll = atan2(
      2.0 * (quaternion[0] * quaternion[1] + quaternion[2] * quaternion[3]),
      1.0 - 2.0 * (quaternion[1] * quaternion[1] + cy));
  sy = sin(pitch);
  cy = cos(pitch);
  sx = sin(roll);
  cx = cos(roll);
  uv[0] = quaternion[2] * linearLocal[2] - linearLocal[1] * quaternion[3];
  uv[1] = linearLocal[0] * quaternion[3] - quaternion[1] * linearLocal[2];
  uv[2] = quaternion[1] * linearLocal[1] - linearLocal[0] * quaternion[2];
  cy = sx * angularLocal[1] / cy + cx * angularLocal[2] / cy;
  r = _mm_loadu_pd(&uv[0]);
  _mm_storeu_pd(&uv[0], _mm_add_pd(r, r));
  _mm_storeu_pd(&state[0], _mm_loadu_pd(&position[0]));
  d = uv[2] + uv[2];
  state[2] = position[2];
  state[3] = yaw;
  state[4] = pitch;
  state[5] = roll;
  state[29] = (linearLocal[0] + quaternion[0] * uv[0]) +
              (quaternion[2] * d - uv[1] * quaternion[3]);
  state[30] = (linearLocal[1] + quaternion[0] * uv[1]) +
              (uv[0] * quaternion[3] - quaternion[1] * d);
  state[31] = (linearLocal[2] + quaternion[0] * d) +
              (quaternion[1] * uv[1] - uv[0] * quaternion[2]);
  state[32] = cy;
  state[33] = cx * angularLocal[1] - sx * angularLocal[2];
  state[34] = angularLocal[0] + sy * cy;
  for (int i{0}; i < 23; i++) {
    state[i + 6] = q[i];
    state[i + 35] = qd[i];
  }
}

//
// File trailer for prepareState.cpp
//
// [EOF]
//
