project(golems_00_defaults)

#-----------------------------------------------
#                COMPILE SETTINGS
#-----------------------------------------------
set(CMAKE_BUILD_TYPE "Debug")
set(BUILD_FLAGS "-std=gnu++0x -Wreturn-type -Wno-enum-compare -Wall")


#-----------------------------------------------
#                FIND PACKAGES
#-----------------------------------------------
# Find QT
find_package(Qt4 COMPONENTS QtCore QtGui Qt3Support QtXml REQUIRED)
set( QT_USE_QTOPENGL TRUE )
add_definitions(${QT_DEFINITIONS})
ADD_DEFINITIONS(-DQT_PLUGIN)
ADD_DEFINITIONS(-DQT_NO_DEBUG)
ADD_DEFINITIONS(-DQT_SHARED)
include(${QT_USE_FILE})


# Find DART
list(APPEND CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_INSTALL_PREFIX}/share/dart")
find_package(DART REQUIRED)
include_directories(${DART_INCLUDE_DIRS})
link_directories(${DART_LIBRARY_DIRS})

# GRIP
find_package(GRIP REQUIRED)
include_directories(${GRIP_INCLUDE_DIRS})

# PCL
find_package( PCL )
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

# Alvar
find_package( alvar )
include_directories( ${ALVAR_INCLUDE_DIRS} )

# OpenCV
set( OpenCV_DIR /home/ana/local/share/OpenCV )
set( OpenCV_CONFIG_PATH /home/ana/local/share/OpenCV )
find_package( OpenCV 2.4.8 EXACT REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

#-----------------------------------------------
#      INCLUDES & SUBDIRECTORY LIBRARIRES
#-----------------------------------------------
# Add source and header files
file(GLOB SRCS testingGrip.cpp)

# Add motionControl
set( golems_DIR $ENV{HOME}/local/share )
find_package(golems REQUIRED)
include_directories( ${golems_INCLUDE_DIRS} )
link_directories( ${golems_LIBRARY_DIRS} )

# Add aoi
set( GM_DIR $ENV{HOME}/local/share )
find_package(GM REQUIRED)
include_directories( ${GM_INCLUDE_DIRS} )
link_directories( ${GM_LIBRARY_DIRS} )

# Add Qt Widgets library
add_library( testingGrip testingGrip.cpp ) # ${SRCS} )
target_link_libraries( testingGrip   ${QT_LIBRARIES}    ${DART_LIBRARIES}  ${GRIP_LIBRARIES} ) # ${PCL_LIBRARIES}
set_target_properties( testingGrip PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin 
ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ) 

add_library( test_alvar3Daccuracy test_alvar3Daccuracy.cpp ) # ${SRCS} )
target_link_libraries( test_alvar3Daccuracy ${OpenCV_LIBRARIES} ${ALVAR_LIBRARIES}  ${QT_LIBRARIES}    ${DART_LIBRARIES}  ${GRIP_LIBRARIES} ) 
set_target_properties( test_alvar3Daccuracy PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin 
ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ) 

add_executable( getMarkersRobotPose getMarkersRobotPose.cpp ) 
target_link_libraries( getMarkersRobotPose aoi_utils ${OpenCV_LIBRARIES} ${ALVAR_LIBRARIES}  ${QT_LIBRARIES}    ${DART_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} ${golems_LIBRARIES} ) 
set_target_properties( getMarkersRobotPose PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin 
ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ) 

# Set project libraries variable
set(project_libs testingGrip )
