# Standalone numerical MEX adapters: no controller, OCS2 or Pinocchio dependency.
cmake_minimum_required(VERSION 3.16)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(g1_matlab_adapters LANGUAGES C CXX)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
  option(G1_BUILD_MATLAB "Build the standalone numerical MEX adapters" ON)
  if(G1_BUILD_MATLAB)
    find_package(Eigen3 REQUIRED)
    find_package(Matlab REQUIRED)
  endif()
endif()

# No MATLAB-independent helper libraries remain: each MEX owns its implementation.
if(NOT G1_BUILD_MATLAB)
  return()
endif()

matlab_add_mex(NAME g1_observation_mex SRC ObservationMex.cpp
  LINK_TO Eigen3::Eigen)
matlab_add_mex(NAME g1_observation_extract_mex SRC ObservationExtractionMex.cpp
  LINK_TO Eigen3::Eigen)
matlab_add_mex(NAME g1_reference_extract_mex SRC ReferenceExtractionMex.cpp)
