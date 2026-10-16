/**
 * @file
 * @brief 独立参考检查和 MEX 入口；通过检查后原样返回428元素参考。
 * 不生成参考、不保存状态、不依赖控制器或 OCS2。
 */
#include <mex.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace {

/** @brief 检查参考发布标志、目标节点、接触时间表及固定摆腿配置。 */
std::array<double, 428> extractReference(const std::array<double, 428>& a) {
  for (double v : a)
    if (!std::isfinite(v))
      throw std::runtime_error("Nonfinite reference field");
  if (a[1] != 0 && a[1] != 1)
    throw std::runtime_error("Invalid reference publication flag");

  // 未发布槽不携带有效目标，保留其完整传输数据。
  if (a[1] == 0)
    return a;
  if (a[0] < 0 || a[0] > 9007199254740992. || a[0] != std::floor(a[0]))
    throw std::runtime_error("Invalid reference tick");
  if ((a[2] != 1 && a[2] != 3) || a[285] < 1 || a[285] > 64 ||
      a[285] != std::floor(a[285]))
    throw std::runtime_error("Reference shape rejected");

  // 目标时间严格递增，接触事件保持非递减；模式数为事件数加一。
  const auto n = static_cast<size_t>(a[2]), m = static_cast<size_t>(a[285]);
  for (size_t k = 1; k < n; ++k)
    if (a[3 + k] <= a[2 + k])
      throw std::runtime_error("Target time order rejected");
  if (!std::is_sorted(a.begin() + 286, a.begin() + 286 + m))
    throw std::runtime_error("Event order rejected");
  for (size_t k = 0; k <= m; ++k)
    if (a[350 + k] < 0 || a[350 + k] > 3 || a[350 + k] != std::floor(a[350 + k]))
      throw std::runtime_error("Unknown contact mode");
  if (a[415] <= a[286 + m - 1])
    throw std::runtime_error("Terminal marker must follow events");

  const std::array<double, 8> swing = {.05, -0.0, .08, -.001, .4, -.15, .3, .005};
  if (!std::equal(swing.begin(), swing.end(), a.begin() + 416))
    throw std::runtime_error("Swing parameters changed");

  // 不对目标值、事件时间或指令做浮点运算，返回值保留原始位模式。
  return a;
}

} // namespace

/** @brief 检查 MATLAB 容器类型，再调用参考检查；入口不包含控制器逻辑。 */
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
  try {
    if (nrhs != 1 || nlhs != 1 || !mxIsDouble(prhs[0]) || mxIsComplex(prhs[0]) ||
        mxIsSparse(prhs[0]) || mxGetNumberOfElements(prhs[0]) != 428)
      throw std::runtime_error("Expected one real dense reference428 and one output");
    std::array<double, 428> raw;
    std::copy_n(mxGetPr(prhs[0]), 428, raw.begin());
    const auto reference = extractReference(raw);
    plhs[0] = mxCreateDoubleMatrix(428, 1, mxREAL);
    std::copy(reference.begin(), reference.end(), mxGetPr(plhs[0]));
  } catch (const std::exception& e) {
    mexErrMsgIdAndTxt("G1:ReferenceExtractionMex", "%s", e.what());
  }
}
