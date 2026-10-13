#pragma once
// Project-owned standalone definitions for the scalar types used by the
// retained MATLAB-authored numerical sources. No MATLAB SDK/runtime required.
// Linux ABI equivalence to the previously used SDK is checked before migration.
#include <cstdint>
using boolean_T = std::uint8_t;
using int64_T = std::int64_t;
using uint64_T = std::uint64_t;
using real_T = double;
using real32_T = float;
