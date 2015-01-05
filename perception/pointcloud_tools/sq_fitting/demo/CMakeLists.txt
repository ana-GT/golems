# CMake for demo
cmake_minimum_required( VERSION 2.8 )

project( demo )

# Source files
include_directories( ${CMAKE_SOURCE_DIR} )

# OpenCV
#find_package( OpenCV REQUIRED )
#include_directories( ${OpenCV_INCLUDE_DIRS} )


# Include this . directory
include_directories( ${PROJECT_SOURCE_DIR} )

# Add subdirectories with libraries required
add_subdirectory( tabletop_segmentation )
add_subdirectory( tabletop_symmetry )


# Add demo files
add_executable( demo demo.cpp )
target_link_libraries( demo tabletop_segmentation mindGapper ${PCL_LIBRARIES} ${OpenCV_LIBS} )

set_target_properties( demo PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )
