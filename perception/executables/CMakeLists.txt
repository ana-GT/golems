#******************
# Executables
#******************
project( executables )

# OpenCV
set( OpenCV_DIR /home/ana/Software/master/share/OpenCV )
set( OpenCV_CONFIG_PATH /home/ana/Software/master/share/OpenCV )
find_package( OpenCV 3.0  EXACT REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Include
include_directories( .. )

# Apps
add_executable( grab_data grab_data.cpp )
target_link_libraries( grab_data ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} lapack blas gfortran )
set_target_properties( grab_data PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( send_data send_data.cpp )
target_link_libraries( send_data perception_msgs ${OpenCV_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} lapack blas gfortran )
set_target_properties( send_data PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


#add_executable( eye_v0 eye_v0.cpp )
#target_link_libraries( eye_v0 perception_msgs tabletop_segmentation ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} lapack blas gfortran )
#set_target_properties( eye_v0 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
#  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

#include_directories(box_detector/src/pcl_tools/include)
#include_directories(box_detector/src/density/include)
#include_directories(box_detector/src/shape_completion/include)
#include_directories(box_detector/src/mvobbox/include)


#add_executable( object_in_box object_in_box.cpp )
#target_link_libraries( object_in_box  perception_msgs tabletop_segmentation container_detection mvobbox pcl_tools ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} lapack blas gfortran )
#set_target_properties( object_in_box PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
#  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin COMPILE_FLAGS -std=gnu++11 )


#add_executable( eye_v1 eye_v1.cpp )
#target_link_libraries( eye_v1 perception_msgs tabletop_segmentation ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} lapack blas gfortran )
#set_target_properties( eye_v1 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
#  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


# FOR EYE_V2
#include_directories( /home/ana/Research/Code/GM )
#include_directories( /home/ana/Research/Code/GM/aoi/refresher/sq_fitting/include )
#include_directories( /home/ana/Research/Code/GM/aoi/refresher/sq_fitting )
#include_directories( /home/ana/Research/Code/GM/aoi/refresher/)
#link_directories( /home/ana/Research/Code/GM/lib )

#add_executable( eye_v2 eye_v2.cpp )
#target_link_libraries( eye_v2  tabletop_segmentation SQ_utils SQ_fitter mindGapper dt Refresher utils  pcl_common pcl_io pcl_kdtree pcl_visualization pcl_search ${OpenCV_LIBRARIES} )
#set_target_properties( eye_v2 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
#  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


#add_executable( getSegmentedClouds getSegmentedClouds.cpp )
#target_link_libraries( getSegmentedClouds  tabletop_segmentation  pcl_common pcl_io pcl_kdtree pcl_visualization pcl_search ${OpenCV_LIBRARIES} )
#set_target_properties( getSegmentedClouds PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
#  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


# Apps
#add_executable( segmentedCloudTestReceiver segmentedCloudTestReceiver.cpp )
#target_link_libraries( segmentedCloudTestReceiver perception_msgs  ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} lapack blas gfortran )
#set_target_properties( segmentedCloudTestReceiver PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
#  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


