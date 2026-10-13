# Official OCS2 currently prints the Pinocchio-2 Frame::parent field.
# Pinocchio 3 renamed that field to parentJoint. Generate a single translation
# unit with this diagnostic-only accessor changed; never edit the submodule.
set(G1_PINOCCHIO_INTERFACE_COMPAT_SOURCE
  "${G1_OCS2_SOURCE_DIR}/ocs2_pinocchio/ocs2_pinocchio_interface/src/PinocchioInterface.cpp")
if(pinocchio_VERSION VERSION_GREATER_EQUAL 3)
  file(READ "${G1_PINOCCHIO_INTERFACE_COMPAT_SOURCE}" g1_pinocchio_interface_source)
  string(FIND "${g1_pinocchio_interface_source}" "model.frames[k].parent;" g1_parent_accessor)
  if(g1_parent_accessor LESS 0)
    message(FATAL_ERROR "Pinned OCS2 diagnostic accessor changed; review compatibility adapter")
  endif()
  string(REPLACE "model.frames[k].parent;" "model.frames[k].parentJoint;"
    g1_pinocchio_interface_source "${g1_pinocchio_interface_source}")
  set(G1_PINOCCHIO_INTERFACE_COMPAT_SOURCE "${CMAKE_BINARY_DIR}/compat/PinocchioInterface.cpp")
  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/compat")
  file(CONFIGURE OUTPUT "${G1_PINOCCHIO_INTERFACE_COMPAT_SOURCE}"
    CONTENT "${g1_pinocchio_interface_source}" @ONLY)
endif()

# Same API rename in the optional upstream centroidal conversion library.
set(G1_CENTROIDAL_COMPAT_SOURCE "${G1_OCS2_SOURCE_DIR}/ocs2_pinocchio/ocs2_centroidal_model/src/CentroidalModelRbdConversions.cpp")
if(pinocchio_VERSION VERSION_GREATER_EQUAL 3)
  file(READ "${G1_CENTROIDAL_COMPAT_SOURCE}" g1_centroidal_source)
  string(REPLACE "model.frames[frameIndex].parent;" "model.frames[frameIndex].parentJoint;"
    g1_centroidal_source "${g1_centroidal_source}")
  set(G1_CENTROIDAL_COMPAT_SOURCE "${CMAKE_BINARY_DIR}/compat/CentroidalModelRbdConversions.cpp")
  file(CONFIGURE OUTPUT "${G1_CENTROIDAL_COMPAT_SOURCE}" CONTENT "${g1_centroidal_source}" @ONLY)
endif()
