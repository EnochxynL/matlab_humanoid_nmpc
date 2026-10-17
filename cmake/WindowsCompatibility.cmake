# Generate platform-only adaptations outside pinned submodule checkouts.
if(NOT WIN32)
  return()
endif()
set(G1_WINDOWS_COMPAT "${CMAKE_BINARY_DIR}/compat/windows")
file(MAKE_DIRECTORY "${G1_WINDOWS_COMPAT}")
if(TARGET Eigen3::Eigen AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9)
  # GCC 8 ICEs on Eigen's SSE asm constraint for non-vectorizable AD scalars.
  # Keep the barrier for vector packets; no scalar arithmetic is replaced.
  find_path(_g1_eigen_root Eigen/Core HINTS ${Eigen3_INCLUDE_DIRS} REQUIRED)
  file(COPY "${_g1_eigen_root}/Eigen" DESTINATION "${G1_WINDOWS_COMPAT}/staging")
  set(_kernel "${G1_WINDOWS_COMPAT}/staging/Eigen/src/Core/products/GeneralBlockPanelKernel.h")
  file(READ "${_kernel}" _kernel_text)
  string(REPLACE "__asm__  (\"\" : \"+x,m\" (*A0));"
    "if constexpr (GEBPTraits::Vectorizable) { __asm__ (\"\" : \"+x,m\" (*A0)); }"
    _kernel_text "${_kernel_text}")
  file(CONFIGURE OUTPUT "${_kernel}" CONTENT "${_kernel_text}" @ONLY)
  file(GLOB_RECURSE _eigen_headers LIST_DIRECTORIES false RELATIVE "${G1_WINDOWS_COMPAT}/staging"
    "${G1_WINDOWS_COMPAT}/staging/Eigen/*")
  foreach(_relative IN LISTS _eigen_headers)
    configure_file("${G1_WINDOWS_COMPAT}/staging/${_relative}"
      "${G1_WINDOWS_COMPAT}/include/${_relative}" COPYONLY)
  endforeach()
endif()
set(_cg "${G1_OCS2_SOURCE_DIR}/ocs2_thirdparty/include/cppad/cg")
file(COPY "${_cg}" DESTINATION "${G1_WINDOWS_COMPAT}/staging/cppad")
set(_dst "${G1_WINDOWS_COMPAT}/staging/cppad/cg")

configure_file("${PROJECT_SOURCE_DIR}/cmake/windows/CppadSystem.hpp"
  "${_dst}/model/system/linux_system.hpp" COPYONLY)
foreach(_header declare_cg.hpp model/dynamic_lib/linux/linux_dynamiclib_model.hpp
    model/dynamic_lib/linux/linux_dynamiclib.hpp model/dynamic_lib/linux/linux_dynamic_model_library_processor.hpp)
  file(READ "${_dst}/${_header}" _text)
  string(REPLACE "#if CPPAD_CG_SYSTEM_LINUX" "#if CPPAD_CG_SYSTEM_LINUX || CPPAD_CG_SYSTEM_WIN" _text "${_text}")
  string(REPLACE "#include <dlfcn.h>" "#include <windows.h>" _text "${_text}")
  if(_header MATCHES "linux_dynamiclib.hpp$")
    string(REPLACE "int dlOpenMode = RTLD_NOW" "int dlOpenMode = 0" _text "${_text}")
    string(REPLACE "if (dynLibName[0] == '/')" "if (system::isAbsolutePath(dynLibName))" _text "${_text}")
    string(REPLACE "dlopen(path.c_str(), dlOpenMode)" "reinterpret_cast<void*>(LoadLibraryA(path.c_str()))" _text "${_text}")
    string(REPLACE " + dlerror()" " + std::to_string(GetLastError())" _text "${_text}")
    string(REPLACE "dlsym(_dynLibHandle, functionName.c_str())" "reinterpret_cast<void*>(GetProcAddress(reinterpret_cast<HMODULE>(_dynLibHandle), functionName.c_str()))" _text "${_text}")
    string(REPLACE "char *err = dlerror();" "const char* err = functor ? nullptr : \"Windows symbol not found\";" _text "${_text}")
    string(REPLACE "dlclose(_dynLibHandle)" "FreeLibrary(reinterpret_cast<HMODULE>(_dynLibHandle))" _text "${_text}")
  endif()
  file(CONFIGURE OUTPUT "${_dst}/${_header}" CONTENT "${_text}" @ONLY)
endforeach()
file(READ "${_dst}/model/functor_generic_model.hpp" _text)
string(REPLACE "#ifdef CPPAD_CG_SYSTEM_LINUX" "#if CPPAD_CG_SYSTEM_LINUX || CPPAD_CG_SYSTEM_WIN" _text "${_text}")
file(CONFIGURE OUTPUT "${_dst}/model/functor_generic_model.hpp" CONTENT "${_text}" @ONLY)
file(READ "${_dst}/model/compiler/gcc_compiler.hpp" _text)
string(REPLACE "/usr/bin/gcc" "${CMAKE_C_COMPILER}" _text "${_text}")
string(REPLACE "this->_compileLibFlags.push_back(\"-rdynamic\");" "this->_compileLibFlags.push_back(\"-Wl,--export-all-symbols\");" _text "${_text}")
string(REPLACE "#if CPPAD_CG_SYSTEM_APPLE" "#if CPPAD_CG_SYSTEM_WIN\n        std::string linkerName;\n#elif CPPAD_CG_SYSTEM_APPLE" _text "${_text}")
string(REPLACE "std::string linkerFlags = \"-Wl,\" + linkerName + \",\" + system::filenameFromPath(library);"
  "std::string linkerFlags = \"-Wl,--export-all-symbols\";" _text "${_text}")
file(CONFIGURE OUTPUT "${_dst}/model/compiler/gcc_compiler.hpp" CONTENT "${_text}" @ONLY)
file(READ "${_dst}/model/compiler/clang_compiler.hpp" _text)
string(REPLACE "#if CPPAD_CG_SYSTEM_APPLE" "#if CPPAD_CG_SYSTEM_WIN\n        std::string linkerName;\n#elif CPPAD_CG_SYSTEM_APPLE" _text "${_text}")
string(REPLACE "std::string linkerFlags = \"-Wl,\" + linkerName + \",\" + system::filenameFromPath(library);"
  "std::string linkerFlags = \"-Wl,--export-all-symbols\";" _text "${_text}")
file(CONFIGURE OUTPUT "${_dst}/model/compiler/clang_compiler.hpp" CONTENT "${_text}" @ONLY)
# Upstream uses unsigned long for generated indices and size_t for host indices.
# LLP64 Windows needs 64-bit types on BOTH sides of the generated C ABI.
# Only the generated platform copy is changed; numerical expressions are untouched.
file(GLOB_RECURSE _headers "${_dst}/*.hpp")
foreach(_header IN LISTS _headers)
  file(READ "${_header}" _text)
  string(REPLACE "unsigned long" "unsigned long long" _text "${_text}")
  file(CONFIGURE OUTPUT "${_header}" CONTENT "${_text}" @ONLY)
endforeach()

# Only publish changed headers; reconfiguration must not rebuild every AD user.
file(GLOB_RECURSE _generated_headers LIST_DIRECTORIES false RELATIVE "${_dst}" "${_dst}/*")
foreach(_relative IN LISTS _generated_headers)
  configure_file("${_dst}/${_relative}"
    "${G1_WINDOWS_COMPAT}/include/cppad/cg/${_relative}" COPYONLY)
endforeach()

set(G1_CPPAD_INTERFACE_SOURCE "${G1_OCS2_SOURCE_DIR}/ocs2_core/src/automatic_differentation/CppAdInterface.cpp")
file(READ "${G1_CPPAD_INTERFACE_SOURCE}" _text)
string(REPLACE "dynamicLib_ = libraryProcessor.createDynamicLibrary(gccCompiler);"
  "libraryProcessor.createDynamicLibrary(gccCompiler, false);" _text "${_text}")
string(REPLACE "model_ = dynamicLib_->model(modelName_);\n\n  setSparsityNonzeros();" "" _text "${_text}")
set(_rename "boost::filesystem::rename(libraryName_ + tmpName_ + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION,\n                            libraryName_ + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION);")
string(REPLACE "${_rename}" "${_rename}\n  loadModels(verbose);" _text "${_text}")
string(REPLACE "compiler.addCompileLibFlag(\"-rdynamic\");" "compiler.addCompileLibFlag(\"-Wl,--export-all-symbols\");" _text "${_text}")
set(G1_CPPAD_INTERFACE_SOURCE "${G1_WINDOWS_COMPAT}/CppAdInterface.cpp")
file(CONFIGURE OUTPUT "${G1_CPPAD_INTERFACE_SOURCE}" CONTENT "${_text}" @ONLY)

# Winpthreads supplies the POSIX thread handle and priority API for this toolchain.
include_directories(BEFORE SYSTEM "${G1_WINDOWS_COMPAT}/include")
add_compile_definitions(NOMINMAX)
add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wa,-mbig-obj>)
