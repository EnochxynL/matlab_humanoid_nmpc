add_library(g1_controller_core SHARED
  controller/G1Controller.cpp
  controller/PlannerContext.cpp
  controller/PlannerExecutor.cpp
  controller/data/LoadedLibraries.cpp)
set_target_properties(g1_controller_core PROPERTIES OUTPUT_NAME g1_controller)
if(WIN32)
  set_target_properties(g1_controller_core PROPERTIES PREFIX "")
endif()
target_include_directories(g1_controller_core
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/controller>"
         "$<INSTALL_INTERFACE:include>"
  PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}"
          "${CMAKE_CURRENT_SOURCE_DIR}/problem")
target_link_libraries(g1_controller_core PRIVATE humanoid_wb_mpc)
if(UNIX AND NOT APPLE)
  target_link_options(g1_controller_core PRIVATE -Wl,--disable-new-dtags)
  set_target_properties(g1_controller_core PROPERTIES BUILD_RPATH "${CMAKE_BINARY_DIR}/lib;$ORIGIN")
endif()
install(TARGETS g1_controller_core LIBRARY DESTINATION lib RUNTIME DESTINATION bin ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES controller/G1Controller.h DESTINATION include)
option(G1_BUILD_TESTS "Build the public-API standalone replay driver" OFF)
if(G1_BUILD_TESTS)
  if(WIN32)
    add_executable(g1_windows_ocs2_ad_smoke "${PROJECT_SOURCE_DIR}/tests/windows/ocs2_ad_smoke.cpp")
    target_link_libraries(g1_windows_ocs2_ad_smoke PRIVATE ocs2_core)
  endif()
  add_executable(g1_core_replay "${PROJECT_SOURCE_DIR}/tests/core_replay.cpp")
  target_link_libraries(g1_core_replay PRIVATE g1_controller_core)
  target_include_directories(g1_core_replay PRIVATE ${Boost_INCLUDE_DIRS})
endif()
