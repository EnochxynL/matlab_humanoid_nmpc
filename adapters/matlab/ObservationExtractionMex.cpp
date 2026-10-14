/** @file 独立观测检查与时间提取；无规划器实例、缓存或求解器依赖。 */
#include "ObservationKernels.h"
#include <mex.h>
#include <algorithm>
#include <stdexcept>
namespace {
template<size_t N> std::array<double,N> read(const mxArray* a) {
  if (!mxIsDouble(a)||mxIsComplex(a)||mxIsSparse(a)||mxGetNumberOfElements(a)!=N)
    throw std::runtime_error("Invalid extraction input type or dimension");
  std::array<double,N> result;
  std::copy_n(mxGetPr(a),N,result.begin());return result;
}
}
void mexFunction(int nlhs,mxArray* plhs[],int nrhs,const mxArray* prhs[]) {
  try {
    if(nrhs!=3||nlhs!=1) throw std::runtime_error("Expected observation74, ocpState58, jointIndices23 and one output");
    const auto identity=g1_controller::observation::extractObservation(read<74>(prhs[0]),read<58>(prhs[1]),read<23>(prhs[2]));
    plhs[0]=mxCreateDoubleMatrix(3,1,mxREAL);
    std::copy(identity.begin(),identity.end(),mxGetPr(plhs[0]));
  } catch(const std::exception& e) {mexErrMsgIdAndTxt("G1:ObservationExtractionMex","%s",e.what());}
}
