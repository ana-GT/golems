# Project: Perception
project( calibration )

# Eigen
include_directories( /usr/include/eigen3 )

# Add project
add_library( calibration calibration_utils.cpp )
#target_link_libraries( calibration )
set_target_properties( calibration PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )
