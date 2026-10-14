#pragma once
#include <cmath>
#include <stdexcept>
// Mathematical seconds supplied by the physical observation, never a host clock.
struct ControlTime {
  double seconds;
  explicit ControlTime(double value):seconds(value) {
    if (!std::isfinite(value) || value < 0) throw std::invalid_argument("Invalid physical control time");
  }
};
