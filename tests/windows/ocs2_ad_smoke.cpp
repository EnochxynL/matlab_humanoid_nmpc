#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <iostream>
#include <stdexcept>
#include <string>

int main(int argc, char** argv) {
  try {
    if (argc != 3) throw std::runtime_error("Usage: smoke cache-folder create|reload");
    ocs2::CppAdInterface model(
        [](const ocs2::ad_vector_t& x, ocs2::ad_vector_t& y) {
          y.resize(1);
          y[0] = x[0] * x[0] + 2 * x[0];
        }, 1, "windows_ocs2_polynomial", argv[1]);
    if (std::string(argv[2]) == "create") {
      model.createModels(ocs2::CppAdInterface::ApproximationOrder::Second, true);
    } else if (std::string(argv[2]) == "reload") {
      model.loadModels(true);
    } else {
      throw std::runtime_error("Unknown operation");
    }
    ocs2::vector_t x(1);
    x[0] = 3;
    const double value = model.getFunctionValue(x)[0];
    const double jacobian = model.getJacobian(x)(0, 0);
    const double hessian = model.getHessian(std::size_t(0), x)(0, 0);
    std::cout << "value=" << value << " jacobian=" << jacobian << " hessian=" << hessian << '\n';
    if (value != 15 || jacobian != 8 || hessian != 2) {
      throw std::runtime_error("OCS2 AD value or derivative mismatch");
    }
    std::cout << "OCS2_AD_PASS " << argv[2] << '\n';
    return 0;
  } catch (const std::exception& exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
}
