/** @file 无状态的观测数值内核；不管理参考、tick、模型或求解器。 */
#pragma once
#include <array>
namespace g1_controller::observation {
std::array<double,3> quaternionToEulerZyx(const std::array<double,4>& wxyz);
std::array<double,3> rotateLinearVelocity(const std::array<double,4>& wxyz, const std::array<double,3>& local);
std::array<double,3> localAngularVelocityToEulerRates(const std::array<double,3>& euler, const std::array<double,3>& omega);
}
