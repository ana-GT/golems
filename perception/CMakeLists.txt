# Project: Perception
project( perception )

# CMake Path
set( CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules" )

# Eigen
include_directories( /usr/include/eigen3 )

# DART
find_package( DART )
include_directories( ${DART_INCLUDE_DIRS} )


# Add LWA4 software paths
# amino, ach and SNS
find_package( amino REQUIRED )
include_directories( ${AMINO_INCLUDE_DIRS} )
find_package( ach REQUIRED )
include_directories( ${ACH_INCLUDE_DIRS} )
find_package( sns REQUIRED )
include_directories( ${SNS_INCLUDE_DIRS} )

# PCL
find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# Libraries
add_subdirectory( msgs )

add_subdirectory( executables )
add_subdirectory( marker_detection )


#add_subdirectory( tabletop_segmentation )
#add_subdirectory( getKinectTran )
#add_subdirectory( box_detector )


