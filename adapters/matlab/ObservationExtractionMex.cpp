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
 * @brief 检查原始观测和 OCP 状态，输出 [tick, canonical time, raw time]。
 * 不持有控制器状态；会话连续性和安装顺序由 ObservationInstall 检查。
 */
#include <mex.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace {

/** @brief 只检查 MATLAB 类型和维度；数值检查集中在 extractObservation。 */
template <size_t N>
std::array<double, N> read(const mxArray* a) {
  if (!mxIsDouble(a) || mxIsComplex(a) || mxIsSparse(a) ||
      mxGetNumberOfElements(a) != N)
    throw std::runtime_error("Invalid extraction input type or dimension");
  std::array<double, N> result;
  std::copy_n(mxGetPr(a), N, result.begin());
  return result;
}

/** @brief 验证观测及复制字段，派生统一的离散物理时刻身份。 */
std::array<double, 3> extractObservation(
    const std::array<double, 74>& a, const std::array<double, 58>& state,
    const std::array<double, 23>& indices) {
  for (double v : a)
    if (!std::isfinite(v))
      throw std::runtime_error("Nonfinite observation");
  for (double v : state)
    if (!std::isfinite(v))
      throw std::runtime_error("Nonfinite OCP state");

  // 原始时间只用于验证与留证，canonical time 始终从整数 tick 派生。
  if (a[0] < 0 || a[0] / .002 > static_cast<double>(uint64_t{1} << 53))
    throw std::runtime_error("Invalid physical time");
  const auto tick = static_cast<uint64_t>(std::llround(a[0] / .002));
  const double canonical = static_cast<double>(tick) * .002;
  if (std::abs(a[0] - canonical) > 1e-12)
    throw std::runtime_error("Raw backend time exceeds canonical physical tick tolerance");

  // 检查真实观测的安全范围；不修改状态或自动修正四元数。
  if (a[3] < .35 || a[3] > 1.3)
    throw std::runtime_error("Base height outside physical safety envelope");
  const double pitch = std::asin(std::clamp(2 * (a[4] * a[6] - a[7] * a[5]), -1., 1.));
  const double roll = std::atan2(2 * (a[4] * a[5] + a[6] * a[7]),
                                 1 - 2 * (a[5] * a[5] + a[6] * a[6]));
  if (std::abs(pitch) > .7853981633974483 || std::abs(roll) > .7853981633974483)
    throw std::runtime_error("Tilt outside physical safety envelope");
  if (std::abs(Eigen::Quaterniond(a[4], a[5], a[6], a[7]).norm() - 1) > 1e-6)
    throw std::runtime_error("Quaternion rejected");

  // indices 是 MATLAB 的1起始索引；位置和关节字段必须逐元素原样复制。
  for (size_t j = 0; j < 3; ++j)
    if (state[j] != a[1 + j])
      throw std::runtime_error("OCP position identity mismatch");
  bool seen[29]{};
  for (size_t j = 0; j < 23; ++j) {
    const double value = indices[j];
    if (!std::isfinite(value) || value < 1 || value > 29 || value != std::floor(value))
      throw std::runtime_error("Invalid joint index");
    const auto k = static_cast<size_t>(value) - 1;
    if (seen[k])
      throw std::runtime_error("Duplicate joint index");
    seen[k] = true;
    if (state[6 + j] != a[14 + k] || state[35 + j] != a[43 + k])
      throw std::runtime_error("OCP joint mapping mismatch");
  }
  return {static_cast<double>(tick), canonical, a[0]};
}

} // namespace

/** @brief 参数转换、单次提取和结果返回；没有初始化或缓存操作。 */
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
  try {
    if (nrhs != 3 || nlhs != 1)
      throw std::runtime_error("Expected observation74, ocpState58, jointIndices23 and one output");
    const auto identity = extractObservation(read<74>(prhs[0]), read<58>(prhs[1]), read<23>(prhs[2]));
    plhs[0] = mxCreateDoubleMatrix(3, 1, mxREAL);
    std::copy(identity.begin(), identity.end(), mxGetPr(plhs[0]));
  } catch (const std::exception& e) {
    mexErrMsgIdAndTxt("G1:ObservationExtractionMex", "%s", e.what());
  }
}
