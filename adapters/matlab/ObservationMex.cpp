/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors
  may be used to endorse or promote products derived from this software
  without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/
/**
 * @file
 * @brief 无状态坐标转换 MEX：wxyz 四元数、世界系线速度及 ZYX Euler 导数。
 * 原数学公式及运算顺序保持不变；所有辅助函数仅供本文件使用。
 */
#include <mex.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace {

std::array<double, 3> copy(const Eigen::Vector3d& v) {
  return {v[0], v[1], v[2]};
}

Eigen::Quaterniond quaternion(const std::array<double, 4>& q) {
  return {q[0], q[1], q[2], q[3]};
}

/** @brief 保留原 ZYX 转换，不增加归一化或截断。 */
std::array<double, 3> quaternionToEulerZyx(const std::array<double, 4>& q) {
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  const double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  const double pitch = std::asin(2.0 * (w * y - z * x));
  const double roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
  return {yaw, pitch, roll};
}

/** @brief 使用局部到世界的姿态旋转线速度。 */
std::array<double, 3> rotateLinearVelocity(
    const std::array<double, 4>& q, const std::array<double, 3>& local) {
  return copy(quaternion(q) * Eigen::Vector3d(local.data()));
}

/** @brief 局部角速度转为 [yaw, pitch, roll] 的时间导数。 */
std::array<double, 3> localAngularVelocityToEulerRates(
    const std::array<double, 3>& euler, const std::array<double, 3>& omega) {
  const double sy = std::sin(euler[1]), cy = std::cos(euler[1]),
               sx = std::sin(euler[2]), cx = std::cos(euler[2]);
  const double wx = omega[0], wy = omega[1], wz = omega[2];
  const double tmp = sx * wy / cy + cx * wz / cy;
  return {tmp, cx * wy - sx * wz, wx + sy * tmp};
}

/** @brief 读取固定维度实数向量，拒绝稀疏、复数和非有限数据。 */
template <size_t N>
std::array<double, N> values(const mxArray* input) {
  if (!mxIsDouble(input) || mxIsComplex(input) || mxIsSparse(input) ||
      mxGetNumberOfElements(input) != N)
    throw std::runtime_error("Invalid kernel input type or dimension");

  std::array<double, N> result;
  std::copy_n(mxGetPr(input), N, result.begin());
  for (double v : result)
    if (!std::isfinite(v))
      throw std::runtime_error("Nonfinite kernel input");
  return result;
}

} // namespace

/** @brief 只负责命令分发与 MATLAB 数组转换。 */
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
  try {
    if (nrhs < 2 || nlhs != 1 || !mxIsChar(prhs[0]))
      throw std::runtime_error("Expected kernel command and one output");

    char* text = mxArrayToString(prhs[0]);
    if (!text)
      throw std::runtime_error("Invalid command");
    const std::string command(text);
    mxFree(text);

    std::array<double, 3> result;
    if (command == "quaternion_to_euler" && nrhs == 2)
      result = quaternionToEulerZyx(values<4>(prhs[1]));
    else if (command == "rotate_linear_velocity" && nrhs == 3)
      result = rotateLinearVelocity(values<4>(prhs[1]), values<3>(prhs[2]));
    else if (command == "angular_velocity_to_euler_rates" && nrhs == 3)
      result = localAngularVelocityToEulerRates(values<3>(prhs[1]), values<3>(prhs[2]));
    else
      throw std::runtime_error("Unknown kernel command or invalid argument count");

    for (double v : result)
      if (!std::isfinite(v))
        throw std::runtime_error("Nonfinite kernel output");
    plhs[0] = mxCreateDoubleMatrix(3, 1, mxREAL);
    std::copy(result.begin(), result.end(), mxGetPr(plhs[0]));
  } catch (const std::exception& e) {
    mexErrMsgIdAndTxt("G1:ObservationMex", "%s", e.what());
  }
}
