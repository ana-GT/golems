cmake_minimum_required( VERSION 2.8 )

set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}  ${CMAKE_SOURCE_DIR}/cmake/Modules )
set( CMAKE_INSTALL_PREFIX $ENV{HOME}/local )
message("CMAKE INSTALL PREFIX: ${CMAKE_INSTALL_PREFIX}" )


link_directories( /usr/lib/bullet_2.82 )
# Eigen
find_package( PkgConfig )
pkg_check_modules( EIGEN REQUIRED eigen3 )
include_directories( ${EIGEN_INCLUDE_DIRS} )

# amino, ach and reflex
find_package( amino REQUIRED )
include_directories( ${AMINO_INCLUDE_DIRS} )
find_package( ach REQUIRED )
include_directories( ${ACH_INCLUDE_DIRS} )
find_package( sns REQUIRED )
include_directories( ${SNS_INCLUDE_DIRS} )

# Generate config file
configure_file( cmake/golemsConfig.cmake.in "${CMAKE_BINARY_DIR}/cmake/golemsConfig.cmake" @ONLY )
install( FILES ${CMAKE_BINARY_DIR}/cmake/golemsConfig.cmake DESTINATION
share )

# Include base directory
include_directories( ${CMAKE_SOURCE_DIR} )

# Subdirectories
add_subdirectory( theremin )
#add_subdirectory( motion_control )
#add_subdirectory( global )
#add_subdirectory( perception )
#add_subdirectory( object_recognition )
#add_subdirectory( projects )
#add_subdirectory( communication )
#add_subdirectory( jsoncpp )

