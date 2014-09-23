# Project: Perception
project( perception )

# CMake Path
set( CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules" )

# Eigen
include_directories( /usr/include/eigen3 )

# DART
find_package( DART )
include_directories( ${DART_INCLUDE_DIRS} )

# OpenCV
find_package( OpenCV )
include_directories( ${OpenCV_INCLUDE_DIRS} )

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
add_subdirectory( tabletop_segmentation )
add_subdirectory( getKinectTran )


# Apps
add_executable( eye_v0 eye_v0.cpp )
target_link_libraries( eye_v0 perception_msgs tabletop_segmentation ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} lapack blas gfortran )
set_target_properties( eye_v0 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( eye_v1 eye_v1.cpp )
target_link_libraries( eye_v1 perception_msgs tabletop_segmentation ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} lapack blas gfortran )
set_target_properties( eye_v1 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


# Apps
add_executable( segmentedCloudTestReceiver segmentedCloudTestReceiver.cpp )
target_link_libraries( segmentedCloudTestReceiver perception_msgs  ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} lapack blas gfortran )
set_target_properties( segmentedCloudTestReceiver PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


