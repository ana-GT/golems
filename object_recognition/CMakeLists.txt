
project( object_recognition )


set( CMAKE_CXX_FLAGS   "-std=gnu++11" )

# Caffe
find_package( Caffe REQUIRED )
include_directories( ${Caffe_INCLUDE_DIRS} )
add_definitions( ${Caffe_DEFINITIONS} )

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_library(  base_classifier SHARED base_classifier.cpp )
target_link_libraries( base_classifier ${Caffe_LIBRARIES} ${OpenCV_LIBRARIES} )
set_target_properties( base_classifier PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

add_library(  objectsDatabase SHARED ObjectsDatabase.cpp base_classifier )
target_link_libraries( objectsDatabase ${Caffe_LIBRARIES} ${OpenCV_LIBRARIES} )
set_target_properties( objectsDatabase PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )
