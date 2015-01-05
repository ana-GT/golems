#******************
# Executables
#******************
project( executables )

# OpenCV
set( OpenCV_DIR /home/ana/local/share/OpenCV )
set( OpenCV_CONFIG_PATH /home/ana/local/share/OpenCV )
find_package( OpenCV  REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# PCL
find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Include
include_directories( ${CMAKE_SOURCE_DIR}/perception )
include_directories( ${CMAKE_SOURCE_DIR}/perception/pointcloud_tools )
include_directories( ${CMAKE_SOURCE_DIR} )

# Apps
add_executable( grab_data grab_data.cpp )
target_link_libraries( grab_data ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} lapack blas gfortran )
set_target_properties( grab_data PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( send_data send_data.cpp )
target_link_libraries( send_data perception_msgs ${OpenCV_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} lapack blas gfortran )
set_target_properties( send_data PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

# Alita's eyes (Kinect version)
add_executable( eye_v0 eye_v0.cpp )
target_link_libraries( eye_v0 perception_msgs tabletop_segmentation ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} lapack blas gfortran )
set_target_properties( eye_v0 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

# Crichton's eyes (PrimeSense 1.09 - Carmine version)
add_executable( crichton_eye_v0 crichton_eye_v0.cpp )
target_link_libraries( crichton_eye_v0 perception_msgs tabletop_segmentation ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} lapack blas gfortran )
set_target_properties( crichton_eye_v0 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( crichton_eye_v1 crichton_eye_v1.cpp )
target_link_libraries( crichton_eye_v1  tabletop_segmentation sq_fitting mindGapper refresher_utils ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${ACH_LIBRARIES} ${SNS_LIBRARIES} )
set_target_properties( crichton_eye_v1 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


#add_executable( eye_v1 eye_v1.cpp )
#target_link_libraries( eye_v1 perception_msgs tabletop_segmentation ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} lapack blas gfortran )
#set_target_properties( eye_v1 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
#  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


#add_executable( crichton_eye_v2 crichton_eye_v2.cpp )
#target_link_libraries( crichton_eye_v2  tabletop_segmentation sq_fitting mindGapper refresher_utils ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} )
#set_target_properties( crichton_eye_v2 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
#  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


#include_directories(box_detector/src/pcl_tools/include)
#include_directories(box_detector/src/density/include)
#include_directories(box_detector/src/shape_completion/include)
#include_directories(box_detector/src/mvobbox/include)


#add_executable( object_in_box object_in_box.cpp )
#target_link_libraries( object_in_box  perception_msgs tabletop_segmentation container_detection mvobbox pcl_tools ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} lapack blas gfortran )
#set_target_properties( object_in_box PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
#  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin COMPILE_FLAGS -std=gnu++11 )




#add_executable( getSegmentedClouds getSegmentedClouds.cpp )
#target_link_libraries( getSegmentedClouds  tabletop_segmentation  pcl_common pcl_io pcl_kdtree pcl_visualization pcl_search ${OpenCV_LIBRARIES} )
#set_target_properties( getSegmentedClouds PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
#  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


# Apps
#add_executable( segmentedCloudTestReceiver segmentedCloudTestReceiver.cpp )
#target_link_libraries( segmentedCloudTestReceiver perception_msgs  ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} lapack blas gfortran )
#set_target_properties( segmentedCloudTestReceiver PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
#  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


