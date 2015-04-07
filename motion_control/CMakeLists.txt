project( motion_control )
set( CMAKE_CXX_FLAGS "-fPIC" )

# Add Tobias' trajectory generation code
add_subdirectory( trajectories )

# Add bimanual msgs
add_subdirectory( msgs )

# Add my motionControl code
add_library( motionControl base_control.cpp base_control.h
  base_dual_control.cpp base_dual_control.h
  Control_Interface.cpp Control_Interface.h
  Arm_Interface.cpp Arm_Interface.h
  Hand_Interface.cpp Hand_Interface.h
  Limb_Interface.cpp Limb_Interface.h
  DualLimb_Interface.cpp DualLimb_Interface.cpp )
target_link_libraries( motionControl tkunz ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} )
set_target_properties( motionControl PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

# Install
install( TARGETS motionControl DESTINATION lib )
install( DIRECTORY  ./  DESTINATION include/golems/motion_control FILES_MATCHING PATTERN "*.h" )

# Trajectory server for one trajectory at a time
add_executable( gatekeeper gatekeeper.cpp )
target_link_libraries( gatekeeper motionControl tkunz ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} rt blas lapack gfortran )
set_target_properties( gatekeeper PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

# Trajectory server for dual trajectories
add_executable( gatekeeper_bimanual gatekeeper_bimanual.cpp )
target_link_libraries( gatekeeper_bimanual motionControl tkunz ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} rt blas lapack gfortran )
set_target_properties( gatekeeper_bimanual PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


# Trajectory server for dual trajectories
add_executable( gatekeeper_alita gatekeeper_alita.cpp )
target_link_libraries( gatekeeper_alita motionControl tkunz ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} rt blas lapack gfortran )
set_target_properties( gatekeeper_alita PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( gatekeeper_crippled gatekeeper_crippled.cpp )
target_link_libraries( gatekeeper_crippled motionControl tkunz ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} rt blas lapack gfortran )
set_target_properties( gatekeeper_crippled PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

# Install
install( TARGETS gatekeeper DESTINATION bin )


add_executable( base_control_server base_control_server.cpp )
target_link_libraries( base_control_server motionControl tkunz ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} rt blas lapack gfortran )
set_target_properties( base_control_server PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

# Client
add_executable( base_control_client base_control_client.cpp )
target_link_libraries( base_control_client motionControl tkunz ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} rt blas lapack gfortran )
set_target_properties( base_control_client PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


