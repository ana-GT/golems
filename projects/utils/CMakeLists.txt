#******************
# Executables
#******************
project( executables )

# Flag for executables using mutex
set( CMAKE_CXX_FLAGS_RELEASE "-std=gnu++11" )

# Caffe ( for Object recognition app = fast_peception_pick_3)
find_package( Caffe )
include_directories( ${Caffe_INCLUDE_DIRS} )


# Qt
find_package( Qt5Core REQUIRED )
 find_package(Qt5Gui REQUIRED)

# OpenCV
find_package( OpenCV  REQUIRED ) # videoio highgui imgproc core )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# OpenNI2 (for Tracker)
find_package( OpenNI2 REQUIRED ) 
#find_package( NiTE2 REQUIRED ) 

#include_directories( ${NiTE2_INCLUDE_DIRS} )
include_directories( ${OpenNI2_INCLUDE_DIRS} )


# PCL
find_package(PCL REQUIRED COMPONENTS common search visualization segmentation surface io )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



# Utilities to get data
add_executable( grab_pointcloud grab_pointcloud.cpp )
target_link_libraries( grab_pointcloud ${PCL_IO_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} )
set_target_properties( grab_pointcloud PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )



# Take picture with joystick
add_executable( joystick_snapshot joystick_snapshot.cpp )
target_link_libraries( joystick_snapshot  tabletop_segmentation  ${PCL_SURFACE_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${OpenCV_LIBRARIES} ${ACH_LIBRARIES} ${SNS_LIBRARIES} )
set_target_properties( joystick_snapshot PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


# Perception Pick
add_executable( perception_pick perception_pick.cpp )
target_link_libraries( perception_pick  perception_msgs tabletop_segmentation sq_fitting mindGapper refresher_utils  ${PCL_SURFACE_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${OpenCV_LIBRARIES} ${ACH_LIBRARIES} ${SNS_LIBRARIES} levmar )
set_target_properties( perception_pick PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )



# Store segmented file
# Store a picked pointcloud with the table normal information in a file
add_executable( store_segmented_file store_segmented_file.cpp )
target_link_libraries( store_segmented_file  perception_msgs tabletop_segmentation sq_fitting mindGapper refresher_utils  ${PCL_SURFACE_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${OpenCV_LIBRARIES} ${ACH_LIBRARIES} ${SNS_LIBRARIES} )
set_target_properties( store_segmented_file PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( store_segmented_group_file store_segmented_group_file.cpp )
target_link_libraries( store_segmented_group_file  perception_msgs tabletop_segmentation sq_fitting mindGapper refresher_utils  ${PCL_SURFACE_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${OpenCV_LIBRARIES} ${ACH_LIBRARIES} ${SNS_LIBRARIES} )
set_target_properties( store_segmented_group_file PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


add_executable( grab_data grab_data.cpp )
target_link_libraries( grab_data ${OpenCV_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} lapack blas gfortran )
set_target_properties( grab_data PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( send_data send_data.cpp )
target_link_libraries( send_data perception_msgs ${OpenCV_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} lapack blas gfortran )
set_target_properties( send_data PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( receive_data receive_data.cpp )
target_link_libraries( receive_data perception_msgs ${OpenCV_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} lapack blas gfortran )
set_target_properties( receive_data PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

# One person tracker
#add_executable( onePerson_tracker onePerson_tracker.cpp )
#target_link_libraries( onePerson_tracker perception_msgs ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} lapack blas gfortran skeleton_tracker )
#set_target_properties( onePerson_tracker PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

# See module
add_executable( see_module_alita see_module_alita.cpp )
target_link_libraries( see_module_alita  perception_msgs tabletop_segmentation sq_fitting mindGapper refresher_utils  ${PCL_SURFACE_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${OpenCV_LIBRARIES} ${ACH_LIBRARIES} ${SNS_LIBRARIES} levmar )
set_target_properties( see_module_alita PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


# See module
add_executable( see_module_v2 see_module_v2.cpp )
target_link_libraries( see_module_v2  tabletop_segmentation sq_fitting mindGapper refresher_utils  ${PCL_SURFACE_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${OpenCV_LIBRARIES} ${ACH_LIBRARIES} ${SNS_LIBRARIES} levmar )
set_target_properties( see_module_v2 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

# Alita's eyes 
add_executable( eye_v0 eye_v0.cpp )
target_link_libraries( eye_v0 perception_msgs tabletop_segmentation ${OpenCV_LIBRARIES} ${PCL_SURFACE_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_COMMON_LIBRARIES} Qt5::Gui Qt5::Core ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} lapack blas gfortran )
set_target_properties( eye_v0 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


add_executable( eye_v1 eye_v1.cpp )
target_link_libraries( eye_v1 perception_msgs tabletop_segmentation ${OpenCV_LIBRARIES} ${PCL_SURFACE_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_COMMON_LIBRARIES} Qt5::Gui Qt5::Core ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} lapack blas gfortran )
set_target_properties( eye_v1 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


# Crichton's eyes
add_executable( crichton_eye_v0 crichton_eye_v0.cpp )
target_link_libraries( crichton_eye_v0 perception_msgs tabletop_segmentation ${OpenCV_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} lapack blas gfortran )
set_target_properties( crichton_eye_v0 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

# Crichton sending 1 superquadric object
add_executable( crichton_eye_v1 crichton_eye_v1.cpp )
target_link_libraries( crichton_eye_v1  tabletop_segmentation sq_fitting mindGapper refresher_utils  ${PCL_SURFACE_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${OpenCV_LIBRARIES} ${ACH_LIBRARIES} ${SNS_LIBRARIES} levmar )
set_target_properties( crichton_eye_v1 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( store_segmented_pointcloud store_segmented_pointcloud.cpp )
target_link_libraries( store_segmented_pointcloud  tabletop_segmentation sq_fitting mindGapper refresher_utils  ${PCL_SURFACE_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${OpenCV_LIBRARIES} ${ACH_LIBRARIES} ${SNS_LIBRARIES} )
set_target_properties( store_segmented_pointcloud PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


###########################################################
## FFTS
###########################################################
# Simple segmentation
add_executable( ftts_segmentation_demo_1 ftts_segmentation_demo_1.cpp )
target_link_libraries( ftts_segmentation_demo_1 ftts )
set_target_properties( ftts_segmentation_demo_1 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ) 

# Segmentation + bounding box
add_executable( ftts_segmentation_demo_2 ftts_segmentation_demo_2.cpp )
target_link_libraries( ftts_segmentation_demo_2 ftts )
set_target_properties( ftts_segmentation_demo_2 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ) 

# Segmentation + bounding box + recognition
add_executable( fast_perception_pick_3 fast_perception_pick_3.cpp classifier.cpp )
target_link_libraries( fast_perception_pick_3 ftts ${Caffe_LIBRARIES} )
set_target_properties( fast_perception_pick_3 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ) 

# Segmentation + bounding box + get training data for OR 
add_executable( ftts_get_object_data_4 ftts_get_object_data_4.cpp )
target_link_libraries( ftts_get_object_data_4 ftts )
set_target_properties( ftts_get_object_data_4 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ) 


# Segmentation + bounding box + get training data for OR 
#add_executable( test_xyz_img test_xyz_img.cpp )
#target_link_libraries( test_xyz_img ${OpenCV_LIBRARIES} )
#set_target_properties( test_xyz_img PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

 
# LCCP 
#add_executable( example_lccp_segmentation example_lccp_segmentation.cpp )
#target_link_libraries( example_lccp_segmentation ${PCL_LIBRARIES} )
#set_target_properties( example_lccp_segmentation  PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ) 

