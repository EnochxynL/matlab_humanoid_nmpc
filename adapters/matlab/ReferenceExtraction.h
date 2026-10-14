/** @file 独立参考提取和检查；固定容量信号保持原始数值，不依赖 OCS2。 */
#pragma once
#include <array>
namespace g1::matlab {
std::array<double,428> extractReference(const std::array<double,428>& raw);
}
