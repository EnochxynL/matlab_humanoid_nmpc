#include <cppad/cg.hpp>
#include <iostream>

// Exercise source generation, native gcc invocation, DLL symbols and fresh reload.
int main(int argc, char** argv) {
  try {
    const std::string folder = CppAD::cg::system::getWorkingDirectory() + "/AD cache with spaces";
    const std::string name = folder + "/polynomial";
    if (argc == 1) {
      using CG = CppAD::cg::CG<double>;
      using AD = CppAD::AD<CG>;
      std::vector<AD> x(1), y(1);
      x[0] = 3.;
      CppAD::Independent(x);
      y[0] = x[0] * x[0] + 2. * x[0];
      CppAD::ADFun<CG> function(x,y);
      CppAD::cg::ModelCSourceGen<double> source(function,"polynomial");
      source.setCreateJacobian(true);
      source.setCreateSparseJacobian(true);
      source.setCreateSparseHessian(true);
      CppAD::cg::ModelLibraryCSourceGen<double> library(source);
      CppAD::cg::GccCompiler<double> compiler;
      CppAD::cg::system::createFolder(folder);
      compiler.setTemporaryFolder(name + "_tmp");
      compiler.setSourcesFolder(name + "_sources");
      compiler.setSaveToDiskFirst(true);
      CppAD::cg::DynamicModelLibraryProcessor<double> processor(library,name);
      processor.createDynamicLibrary(compiler,false);
    }
    CppAD::cg::LinuxDynamicLib<double> library(name + ".dll");
    auto model = library.model("polynomial");
    const std::vector<double> x{3.};
    if (model->ForwardZero(x)[0] != 15. || model->Jacobian(x)[0] != 8. ||
        model->SparseJacobian(x)[0] != 8. || model->SparseHessian(x,std::vector<double>{1.})[0] != 2.)
      throw std::runtime_error("AD value, Jacobian or sparse Hessian mismatch");
    std::cout << (argc == 1 ? "AD generation/load PASS" : "AD fresh reload PASS") << std::endl;
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}
