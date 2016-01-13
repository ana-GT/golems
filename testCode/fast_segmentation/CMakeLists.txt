cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(fast_segmentation)

SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -std=c++11")
SET(CMAKE_C_FLAGS_RELEASE "-O3 -std=c++11")

#set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
message (MODULE_PATH= ${CMAKE_MODULE_PATH})

###########################################################################
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

# Find OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

include_directories(include)
set(incs fast_segmentation.h)

set(srcs fast_segmentation.cpp)

add_library(fast_segmentation_tools SHARED ${incs} ${srcs})
target_link_libraries(fast_segmentation_tools ${PCL_LIBRARIES} tbb tbbmalloc)
set_target_properties( fast_segmentation_tools PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

add_executable(fast_segmentation_demo fast_segmentation_demo.cpp)
target_link_libraries(fast_segmentation_demo fast_segmentation_tools)
set_target_properties( fast_segmentation_demo PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ) 

add_library( fast_tts SHARED fast_tabletop_segmentation.cpp fast_tabletop_segmentation.h )
target_link_libraries(fast_tts ${PCL_LIBRARIES} )
set_target_properties( fast_tts PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )




 
add_executable(fast_pick fast_pick.cpp)
target_link_libraries(fast_pick fast_tts ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} )
set_target_properties( fast_pick PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

 
add_executable( pcd_organized_multi_plane_segmentation pcd_organized_multi_plane_segmentation.cpp)
target_link_libraries( pcd_organized_multi_plane_segmentation ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} )
set_target_properties( pcd_organized_multi_plane_segmentation PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ) 

#add_executable( organized_segmentation_demo organized_segmentation_demo.cpp)
#target_link_libraries( organized_segmentation_demo ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} )
#set_target_properties( organized_segmentation_demo PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ) 

add_executable(grab_pointcloud grab_pointcloud.cpp)
target_link_libraries(grab_pointcloud ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} )
set_target_properties( grab_pointcloud PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( test_plane_segmentation test_plane_segmentation.cpp)
target_link_libraries(test_plane_segmentation ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} )
set_target_properties( test_plane_segmentation PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )
