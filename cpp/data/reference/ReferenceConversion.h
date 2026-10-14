/** @file 外部参考与 OCP 参考包的格式转换；不拥有发布顺序或求解生命周期。 */
#pragma once
#include "g1_controller/Types.h"
#include "data/reference/ExternalReferenceAdapter.h"
namespace g1_controller::reference {
std::array<double, 428> encodeReference(const Reference& reference);
task5a::Packet decodeReference(const double* encoded, std::uint64_t sequence);
} // namespace g1_controller::reference
