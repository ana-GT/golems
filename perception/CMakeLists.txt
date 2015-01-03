# Project: Perception
project( perception )

# CMake Path
set( CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules" )

# Eigen
include_directories( /usr/include/eigen3 )

# OpenCV
set( OpenCV_DIR /home/ana/local/share/OpenCV )
set( OpenCV_CONFIG_PATH /home/ana/local/share/OpenCV )
find_package( OpenCV  REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

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
add_subdirectory( calibration )
add_subdirectory( marker_detection )
add_subdirectory( tabletop_segmentation )
#add_subdirectory( getKinectTran )
#add_subdirectory( box_detector )

#Install
install( DIRECTORY msgs DESTINATION include/golems/perception/msgs FILES_MATCHING PATTERN "*.h" )
install( DIRECTORY calibration DESTINATION include/golems/perception/calibration FILES_MATCHING PATTERN "*.h" )
install( DIRECTORY marker_detection DESTINATION include/golems/perception/marker_detection FILES_MATCHING PATTERN "*.h" )
install( DIRECTORY tabletop_segmentation DESTINATION include/golems/perception/tabletop_segmentation FILES_MATCHING PATTERN "*.h" )

