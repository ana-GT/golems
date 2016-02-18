# Project: Perception
project( perception )

# Eigen
include_directories( /usr/include/eigen3 )

# OpenCV
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
find_package(PCL REQUIRED COMPONENTS common search visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# Libraries
add_subdirectory( msgs )
add_subdirectory( calibration )
#add_subdirectory( skeleton_tracker )
add_subdirectory( marker_detection )
add_subdirectory( tabletop_segmentation )
add_subdirectory( pointcloud_tools )
#add_subdirectory( ftts )
#add_subdirectory( getKinectTran )
#add_subdirectory( box_detector )

#Install
install( DIRECTORY msgs DESTINATION include/golems/perception/ FILES_MATCHING PATTERN "*.h" )
install( DIRECTORY calibration DESTINATION include/golems/perception/ FILES_MATCHING PATTERN "*.h" )
install( DIRECTORY marker_detection DESTINATION include/golems/perception/ FILES_MATCHING PATTERN "*.h" )
install( DIRECTORY tabletop_segmentation DESTINATION include/golems/perception/ FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" )
