project( motion_control )

# Find Eigen
find_package( PkgConfig )
pkg_check_modules( EIGEN REQUIRED eigen3 )
include_directories( ${EIGEN_INCLUDE_DIRS} )

add_subdirectory( trajectories )

add_library( motionControl base_control.cpp base_control.h )
target_link_libraries( motionControl tkunz ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} )
set_target_properties( motionControl PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

#Set project library variables
set( project_libs motionControl )