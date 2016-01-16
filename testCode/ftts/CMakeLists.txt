cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ftts)

SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -std=c++11")
SET(CMAKE_C_FLAGS_RELEASE "-O3 -std=c++11")

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

# Find OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Caffe ( for Object recognition app = fast_peception_pick_3)
find_package( Caffe )
include_directories( ${Caffe_INCLUDE_DIRS} )


add_library( ftts SHARED fast_tabletop_segmentation.cpp fast_tabletop_segmentation.h )
target_link_libraries( ftts ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} )
set_target_properties( ftts PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

# Simple segmentation
add_executable( fast_perception_pick fast_perception_pick.cpp )
target_link_libraries( fast_perception_pick ftts )
set_target_properties( fast_perception_pick PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ) 

# Segmentation + bounding box
add_executable( fast_perception_pick_2 fast_perception_pick_2.cpp )
target_link_libraries( fast_perception_pick_2 ftts )
set_target_properties( fast_perception_pick_2 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ) 

# Segmentation + bounding box + recognition
add_executable( fast_perception_pick_3 fast_perception_pick_3.cpp classifier.cpp )
target_link_libraries( fast_perception_pick_3 ftts ${Caffe_LIBRARIES} )
set_target_properties( fast_perception_pick_3 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ) 

# Segmentation + bounding box + get training data for OR 
add_executable( fast_perception_pick_4 fast_perception_pick_4.cpp )
target_link_libraries( fast_perception_pick_4 ftts )
set_target_properties( fast_perception_pick_4 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ) 


# Segmentation + bounding box + get training data for OR 
add_executable( test_xyz_img test_xyz_img.cpp )
target_link_libraries( test_xyz_img ${OpenCV_LIBRARIES} )
set_target_properties( test_xyz_img PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ) 
