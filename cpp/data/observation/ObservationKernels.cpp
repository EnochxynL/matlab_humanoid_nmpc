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
}
