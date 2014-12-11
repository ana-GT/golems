project( motion_control )

# Find Eigen
find_package( PkgConfig )
pkg_check_modules( EIGEN REQUIRED eigen3 )
include_directories( ${EIGEN_INCLUDE_DIRS} )

add_subdirectory( trajectories )

add_library( motionControl base_control.cpp base_control.h )
target_link_libraries( motionControl tkunz ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} )
set_target_properties( motionControl PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

# Servers (to be run in calvin)
add_executable( base_control_server base_control_server.cpp )
target_link_libraries( base_control_server motionControl tkunz ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} )
set_target_properties( base_control_server PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

# Client
add_executable( base_control_client base_control_client.cpp )
target_link_libraries( base_control_client motionControl tkunz ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} )
set_target_properties( base_control_client PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )
