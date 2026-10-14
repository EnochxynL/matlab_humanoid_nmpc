/** @file 独立无状态观测 MEX：仅转换参数并调用小型数值内核。 */
#include "ObservationKernels.h"
#include <mex.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
namespace {
template<size_t N> std::array<double,N> values(const mxArray* input) {
  if (!mxIsDouble(input)||mxIsComplex(input)||mxIsSparse(input)||mxGetNumberOfElements(input)!=N)
    throw std::runtime_error("Invalid kernel input type or dimension");
  std::array<double,N> result;
  std::copy_n(mxGetPr(input),N,result.begin());
  for(double v:result) if(!std::isfinite(v)) throw std::runtime_error("Nonfinite kernel input");
  return result;
}
}
void mexFunction(int nlhs,mxArray* plhs[],int nrhs,const mxArray* prhs[]) {
  try {
    if(nrhs<2||nlhs!=1||!mxIsChar(prhs[0])) throw std::runtime_error("Expected kernel command and one output");
    char* text=mxArrayToString(prhs[0]);if(!text) throw std::runtime_error("Invalid command");
    const std::string command(text);mxFree(text);
    std::array<double,3> result;
    using namespace g1_controller::observation;
    if(command=="quaternion_to_euler"&&nrhs==2) result=quaternionToEulerZyx(values<4>(prhs[1]));
    else if(command=="rotate_linear_velocity"&&nrhs==3) result=rotateLinearVelocity(values<4>(prhs[1]),values<3>(prhs[2]));
    else if(command=="angular_velocity_to_euler_rates"&&nrhs==3) result=localAngularVelocityToEulerRates(values<3>(prhs[1]),values<3>(prhs[2]));
    else throw std::runtime_error("Unknown kernel command or invalid argument count");
    for(double v:result) if(!std::isfinite(v)) throw std::runtime_error("Nonfinite kernel output");
    plhs[0]=mxCreateDoubleMatrix(3,1,mxREAL);std::copy(result.begin(),result.end(),mxGetPr(plhs[0]));
  } catch(const std::exception& e) {mexErrMsgIdAndTxt("G1:ObservationMex","%s",e.what());}
}
