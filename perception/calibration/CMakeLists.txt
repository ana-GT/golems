# Project: Perception
project( calibration )

# Eigen
include_directories( /usr/include/eigen3 )

# Add aoi
set( GM_DIR $ENV{HOME}/local/share )
find_package(GM REQUIRED)
include_directories( ${GM_INCLUDE_DIRS} )
link_directories( ${GM_LIBRARY_DIRS} )

# Find DART
list(APPEND CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_INSTALL_PREFIX}/share/dart")
find_package(DART REQUIRED)
include_directories(${DART_INCLUDE_DIRS})
link_directories(${DART_LIBRARY_DIRS})

# Add motionControl
set( golems_DIR $ENV{HOME}/local/share )
find_package(golems REQUIRED)
include_directories( ${golems_INCLUDE_DIRS} )
link_directories( ${golems_LIBRARY_DIRS} )


# GRIP
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/local/share/cmake")
find_package(GRIP REQUIRED)
include_directories(${GRIP_INCLUDE_DIRS})

# Add project
add_library( calibration calibration_utils.cpp )
set_target_properties( calibration PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

