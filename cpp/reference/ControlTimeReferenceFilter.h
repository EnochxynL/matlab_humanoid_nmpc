/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.
Copyright (c) 2024, 1X Technologies. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <humanoid_common_mpc/common/Types.h>
#include <cmath>
#include <stdexcept>

namespace ocs2::humanoid {

// Task3D approved recurrence and startup/reset contract; explicit control time only.
class ControlTimeReferenceFilter final {
 public:
  ControlTimeReferenceFilter(scalar_t breakFrequency, const vector_t& y_init)
      : breakDeltaT_(1 / (2 * M_PI * breakFrequency)), y_last_(y_init) {}

  // Caller resets at a new control-time epoch. First sample uses the scaled
  // command as the explicit initial condition, never wall-clock waiting.
  void reset(const vector_t& initialValue, scalar_t controlTime) {
    if (!std::isfinite(controlTime)) throw std::invalid_argument("Nonfinite control time");
    y_last_ = initialValue;
    lastControlTime_ = controlTime;
  }

  vector_t getFilteredVector(const vector_t& x, scalar_t controlTime) {
    assert(x.size() == y_last_.size());
    if (!std::isfinite(controlTime) || controlTime < lastControlTime_)
      throw std::invalid_argument("Filter control time must be finite and monotone; reset new epochs explicitly");
    const scalar_t delta = controlTime - lastControlTime_;
    if (delta == 0) return y_last_;  // idempotent repeated solve/observation
    const scalar_t alpha = delta / (delta + breakDeltaT_); // original coefficient
    y_last_ = alpha * x + (1 - alpha) * y_last_;          // proposed math fix
    lastControlTime_ = controlTime;                       // proposed state fix
    return y_last_;
  }

 private:
  scalar_t breakDeltaT_;
  vector_t y_last_;
  scalar_t lastControlTime_ = 0;
};

}  // namespace ocs2::humanoid
