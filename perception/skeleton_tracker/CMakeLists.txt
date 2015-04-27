project( skeleton_tracker )


find_package( OpenNI2 REQUIRED ) 
find_package( NiTE2 REQUIRED ) 
find_package( PkgConfig )
pkg_check_modules( EIGEN eigen3 )


include_directories( ${NiTE2_INCLUDE_DIRS} )
include_directories( ${OpenNI2_INCLUDE_DIRS} )
include_directories( ${EIGEN_INCLUDE_DIRS} )

include_directories( ${CMAKE_SOURCE_DIR}/perception )

add_library( skeleton_tracker skeleton_tracker.cpp skeleton_tracker.h )
target_link_libraries( skeleton_tracker ${NiTE2_LIBS} ${OpenNI2_LIBS} )
set_target_properties( skeleton_tracker PROPERTIES 
		       ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
		       RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )
