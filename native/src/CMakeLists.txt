add_library(g1_controller_core SHARED
  g1_controller/G1Controller.cpp
  g1_controller/controller/ExternalReferenceManager.cpp
  g1_controller/controller/PacketManager.cpp
  g1_controller/controller/Ocs2Backend.cpp)
set_target_properties(g1_controller_core PROPERTIES OUTPUT_NAME g1_controller)
if(WIN32)
  set_target_properties(g1_controller_core PROPERTIES PREFIX "")
endif()
target_include_directories(g1_controller_core
  PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
         "$<INSTALL_INTERFACE:include>"
  PRIVATE "${G1_CORE_DIR}/controller"
          "${G1_CORE_DIR}"
          "${G1_CORE_DIR}/problem")
target_link_libraries(g1_controller_core PRIVATE humanoid_wb_mpc)
if(UNIX AND NOT APPLE)
  target_link_options(g1_controller_core PRIVATE -Wl,--disable-new-dtags)
  set_target_properties(g1_controller_core PROPERTIES BUILD_RPATH "${CMAKE_BINARY_DIR}/lib;$ORIGIN" INSTALL_RPATH "$ORIGIN")
endif()
install(TARGETS g1_controller_core LIBRARY DESTINATION lib RUNTIME DESTINATION lib ARCHIVE DESTINATION lib)
install(DIRECTORY "${PROJECT_SOURCE_DIR}/include/" DESTINATION include)

# Independent inverse-dynamics facade: model only, never the planning targets.
add_library(g1_dynamics_core SHARED
  g1_controller/InverseDynamics.cpp)
set_target_properties(g1_dynamics_core PROPERTIES OUTPUT_NAME g1_dynamics)
if(WIN32)
  set_target_properties(g1_dynamics_core PROPERTIES PREFIX "")
endif()
target_include_directories(g1_dynamics_core PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(g1_dynamics_core PRIVATE humanoid_model)
if(UNIX AND NOT APPLE)
  target_link_options(g1_dynamics_core PRIVATE -Wl,--disable-new-dtags)
  set_target_properties(g1_dynamics_core PROPERTIES
    BUILD_RPATH "${CMAKE_BINARY_DIR}/lib;$ORIGIN" INSTALL_RPATH "$ORIGIN")
endif()
install(TARGETS g1_dynamics_core LIBRARY DESTINATION lib RUNTIME DESTINATION lib ARCHIVE DESTINATION lib)
option(G1_BUILD_TESTS "Build the public-API standalone replay driver" OFF)
if(G1_BUILD_TESTS)
  if(WIN32)
    add_executable(g1_windows_ocs2_ad_smoke "${PROJECT_SOURCE_DIR}/tests/windows/ocs2_ad_smoke.cpp")
    target_link_libraries(g1_windows_ocs2_ad_smoke PRIVATE ocs2_core)
  endif()
  add_executable(g1_core_replay "${PROJECT_SOURCE_DIR}/tests/core_replay.cpp")
  target_link_libraries(g1_core_replay PRIVATE g1_controller_core g1_dynamics_core)
  target_include_directories(g1_core_replay PRIVATE ${Boost_INCLUDE_DIRS})
endif()
