# Original source code from pcl/tutorials/iccv11
project(obj_recog)

# PCL
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# jsoncpp
include_directories( ${CMAKE_SOURCE_DIR}/jsoncpp )

add_library( obj_recog object_recognition.cpp
  load_clouds.cpp
  feature_estimation.cpp
  registration.cpp
  filters.cpp
  segmentation.cpp )
target_link_libraries( obj_recog  jsoncpp ${PCL_LIBRARIES} )
set_target_properties( obj_recog PROPERTIES 
		       ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
		       LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

