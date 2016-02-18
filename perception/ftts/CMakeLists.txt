# Fast Tabletop segmentation
project(ftts)

SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -std=c++11")
SET(CMAKE_C_FLAGS_RELEASE "-O3 -std=c++11")

find_package(  Qt5 REQUIRED Widgets )

# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# ftts library
add_library( ftts SHARED fast_tabletop_segmentation.cpp fast_tabletop_segmentation.h )
target_link_libraries( ftts ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} )
set_target_properties( ftts PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

