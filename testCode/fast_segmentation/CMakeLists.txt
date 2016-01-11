cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(organized_segmentation_tools)

SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -std=c++11")
SET(CMAKE_C_FLAGS_RELEASE "-O3 -std=c++11")

#set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
message (MODULE_PATH= ${CMAKE_MODULE_PATH})

###########################################################################
# Find PCL
find_package(PCL 1.7 REQUIRED)
message( STATUS "Find packages" )
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})


include_directories(include)
set(incs fast_segmentation.h)

set(srcs fast_segmentation.cpp)

add_library(fast_segmentation_tools SHARED ${incs} ${srcs})
target_link_libraries(fast_segmentation_tools ${PCL_LIBRARIES} tbb tbbmalloc)

add_executable(fast_segmentation_demo fast_segmentation_demo.cpp)
target_link_libraries(fast_segmentation_demo fast_segmentation_tools)
