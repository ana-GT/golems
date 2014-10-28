cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project("container_detection")

# Implied by PCL
find_package(Boost REQUIRED COMPONENTS
  filesystem
  program_options
)

find_package(PCL REQUIRED COMPONENTS
  common
  io
  features
  filters
  segmentation
  visualization
)

### Include directories
include_directories(
  include/
  ${PCL_INCLUDE_DIRS}
)

### Container detection library
add_library(container_detection SHARED
  src/container_detection.cpp
)

target_link_libraries(container_detection
  ${PCL_FEATURES_LIBRARIES}
  ${PCL_FILTERS_LIBRARIES}
  ${PCL_SEGMENTATION_LIBRARIES}
  pcl_tools
  mvobbox
)

set_target_properties(container_detection
  PROPERTIES 
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib 
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
)

### Container detection demo
#add_executable(container_detection_demo
#  src/main.cpp
#)

#target_link_libraries(container_detection_demo
#  ${PCL_COMMON_LIBRARIES}
#  ${PCL_IO_LIBRARIES}
#  ${PCL_VISUALIZATION_LIBRARIES}
#  ${Boost_PROGRAM_OPTIONS_LIBRARY}
#  pcl_tools_opencv
#  container_detection
#)

#set_target_properties(container_detection_demo
#  PROPERTIES 
#    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin 
#    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
#)
