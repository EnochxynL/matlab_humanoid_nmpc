/** @file 独立无状态参考 MEX；不创建控制器、不保存参考、不链接 OCS2。 */
#include "ReferenceExtraction.h"
#include <mex.h>
#include <algorithm>
#include <stdexcept>
void mexFunction(int nlhs,mxArray* plhs[],int nrhs,const mxArray* prhs[]) {
  try {
    if(nrhs!=1||nlhs!=1||!mxIsDouble(prhs[0])||mxIsComplex(prhs[0])||mxIsSparse(prhs[0])||mxGetNumberOfElements(prhs[0])!=428)
      throw std::runtime_error("Expected one real dense reference428 and one output");
    std::array<double,428> raw;
    std::copy_n(mxGetPr(prhs[0]),428,raw.begin());
    const auto reference=g1::matlab::extractReference(raw);
    plhs[0]=mxCreateDoubleMatrix(428,1,mxREAL);
    std::copy(reference.begin(),reference.end(),mxGetPr(plhs[0]));
  } catch(const std::exception& e) {mexErrMsgIdAndTxt("G1:ReferenceExtractionMex","%s",e.what());}
}
