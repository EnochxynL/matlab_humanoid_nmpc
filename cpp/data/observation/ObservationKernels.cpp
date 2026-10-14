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
#include "ObservationKernels.h"
#include <Eigen/Geometry>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <cstdint>
namespace g1_controller::observation {
namespace {
std::array<double,3> copy(const Eigen::Vector3d& v) { return {v[0],v[1],v[2]}; }
Eigen::Quaterniond quaternion(const std::array<double,4>& q) { return {q[0],q[1],q[2],q[3]}; }
}
// Original quaternionToEulerZYX formula; wxyz, no normalization or clamp.
std::array<double,3> quaternionToEulerZyx(const std::array<double,4>& q) {
  const double w=q[0],x=q[1],y=q[2],z=q[3];
  const double yaw=std::atan2(2.0*(w*z+x*y),1.0-2.0*(y*y+z*z));
  const double pitch=std::asin(2.0*(w*y-z*x));
  const double roll=std::atan2(2.0*(w*x+y*z),1.0-2.0*(x*x+y*y));
  return {yaw,pitch,roll};
}
std::array<double,3> rotateLinearVelocity(const std::array<double,4>& q, const std::array<double,3>& local) {
  return copy(quaternion(q) * Eigen::Vector3d(local.data()));
}
// Original OCS2 local-angular-velocity conversion, in [yaw,pitch,roll] order.
std::array<double,3> localAngularVelocityToEulerRates(const std::array<double,3>& euler, const std::array<double,3>& omega) {
  const double sy=std::sin(euler[1]),cy=std::cos(euler[1]),sx=std::sin(euler[2]),cx=std::cos(euler[2]);
  const double wx=omega[0],wy=omega[1],wz=omega[2];
  const double tmp=sx*wy/cy+cx*wz/cy;
  return {tmp,cx*wy-sx*wz,wx+sy*tmp};
}
std::array<double,3> extractObservation(const std::array<double,74>& a,
    const std::array<double,58>& state, const std::array<double,23>& indices) {
  for(double v:a) if(!std::isfinite(v)) throw std::runtime_error("Nonfinite observation");
  for(double v:state) if(!std::isfinite(v)) throw std::runtime_error("Nonfinite OCP state");
  if(a[0]<0 || a[0]/.002>static_cast<double>(uint64_t{1}<<53))
    throw std::runtime_error("Invalid physical time");
  const auto tick=static_cast<uint64_t>(std::llround(a[0]/.002));
  const double canonical=static_cast<double>(tick)*.002;
  if(std::abs(a[0]-canonical)>1e-12) throw std::runtime_error("Raw backend time exceeds canonical physical tick tolerance");
  if(a[3]<.35 || a[3]>1.3) throw std::runtime_error("Base height outside physical safety envelope");
  const double pitch=std::asin(std::clamp(2*(a[4]*a[6]-a[7]*a[5]),-1.,1.));
  const double roll=std::atan2(2*(a[4]*a[5]+a[6]*a[7]),1-2*(a[5]*a[5]+a[6]*a[6]));
  if(std::abs(pitch)>.7853981633974483 || std::abs(roll)>.7853981633974483)
    throw std::runtime_error("Tilt outside physical safety envelope");
  if(std::abs(Eigen::Quaterniond(a[4],a[5],a[6],a[7]).norm()-1)>1e-6)
    throw std::runtime_error("Quaternion rejected");
  for(size_t j=0;j<3;++j) if(state[j]!=a[1+j]) throw std::runtime_error("OCP position identity mismatch");
  bool seen[29]{};
  for(size_t j=0;j<23;++j) {
    const double value=indices[j];
    if(!std::isfinite(value)||value<1||value>29||value!=std::floor(value))
      throw std::runtime_error("Invalid joint index");
    const auto k=static_cast<size_t>(value)-1;
    if(seen[k]) throw std::runtime_error("Duplicate joint index");
    seen[k]=true;
    if(state[6+j]!=a[14+k] || state[35+j]!=a[43+k]) throw std::runtime_error("OCP joint mapping mismatch");
  }
  return {static_cast<double>(tick),canonical,a[0]};
}

}
