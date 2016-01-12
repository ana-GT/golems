cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(organized_segmentation_tools)

SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -std=c++11")
SET(CMAKE_C_FLAGS_RELEASE "-O3 -std=c++11")

#set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
message (MODULE_PATH= ${CMAKE_MODULE_PATH})

###########################################################################
# Find PCL
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

include_directories(include)
set(incs include/organized_segmentation_tools/organized_segmentation_tbb.h)

set(srcs src/organized_segmentation_tbb.cpp)

add_library(organized_segmentation_tools SHARED ${incs} ${srcs})
target_link_libraries(organized_segmentation_tools ${PCL_LIBRARIES} tbb tbbmalloc)
set_target_properties( organized_segmentation_tools PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

add_executable(organized_segmentation_tbb_demo src/organized_segmentation_tbb_demo.cpp)
target_link_libraries(organized_segmentation_tbb_demo organized_segmentation_tools)
set_target_properties( organized_segmentation_tbb_demo PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )
