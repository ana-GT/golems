#******************
# Executables
#******************
project( executables )

# Flag for executables using mutex
set( CMAKE_CXX_FLAGS "-std=gnu++11" )

# Caffe ( for Object recognition app = fast_peception_pick_3)
find_package( Caffe )
include_directories( ${Caffe_INCLUDE_DIRS} )


# Qt
find_package( Qt5Core REQUIRED )
 find_package(Qt5Gui REQUIRED)
find_packagE( Qt5Widgets REQUIRED )

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

add_executable( generate_SQ_meshes generate_SQ_meshes.cpp )
target_link_libraries( generate_SQ_meshes  sq_utils  ${PCL_SURFACE_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_FILTERS_LIBRARIES} Qt5::Widgets )
set_target_properties( generate_SQ_meshes PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

# Perception Pick
add_executable( perception_pick perception_pick.cpp )
target_link_libraries( perception_pick  perception_msgs tabletop_segmentation sq_fitting mindGapper refresher_utils  ${PCL_SURFACE_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${OpenCV_LIBRARIES} ${ACH_LIBRARIES} ${SNS_LIBRARIES} levmar )
set_target_properties( perception_pick PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

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

# Perception pick with ftts
add_executable( perception_pick_ftts perception_pick_ftts.cpp classifier.cpp )
target_link_libraries( perception_pick_ftts ftts sq_fitting sq_utils perception_msgs ${Caffe_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES} levmar )
set_target_properties( perception_pick_ftts PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ) 

# Perception pick with ftts
add_executable( perception_pick_ftts_2 perception_pick_ftts_2.cpp )
target_link_libraries( perception_pick_ftts_2 ftts sq_fitting sq_utils objectsDatabase perception_msgs ${Caffe_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES} levmar )
set_target_properties( perception_pick_ftts_2 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ) 
