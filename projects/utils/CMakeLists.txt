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

