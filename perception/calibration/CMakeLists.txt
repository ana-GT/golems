# Project: Perception
project( calibration )

# Add project
add_library( calibration SHARED calibration_utils.cpp )
set_target_properties( calibration PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

