project(bimanualControl)

#-----------------------------------------------
#                COMPILE SETTINGS
#-----------------------------------------------
set(CMAKE_BUILD_TYPE "Release")
set(BUILD_FLAGS "-std=gnu++0x -Wreturn-type -Wno-enum-compare -Wall")
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/build)
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules")
set( CMAKE_CXX_FLAGS "-fPIC" )

# Define preprocessor macro for Debug printing in Debug mode
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "Building in \"Debug\" mode. Defining DEBUG_BUILD macro.")
    add_definitions(-DDEBUG_BUILD)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    message(STATUS "Building in \"Release\" mode.")
else()
endif()

find_package( PkgConfig )
pkg_check_modules( EIGEN REQUIRED eigen3 )
include_directories( ${EIGEN_INCLUDE_DIRS} )


#-----------------------------------------------
#      INCLUDES & SUBDIRECTORY LIBRARIRES
#-----------------------------------------------

# Add LWA4 software paths / include / libraries
# amino, ach and reflex
# Check piranhina
#find_package( amino REQUIRED )
#include_directories( ${AMINO_INCLUDE_DIRS} )
#find_package( ach REQUIRED )
#include_directories( ${ACH_INCLUDE_DIRS} )
#find_package( sns REQUIRED )
#include_directories( ${SNS_INCLUDE_DIRS} )

# Tobias's trajectories
add_subdirectory( trajectories )


# Add Qt Widgets library
add_library( basicControl bimanualControl.cpp bimanualControl.h unimanualControl.cpp unimanualControl.h basicControl.cpp basicControl.h )
target_link_libraries( basicControl tkunz ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} )
set_target_properties( basicControl PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

# Set project libraries variable
set(project_libs basicControl )
