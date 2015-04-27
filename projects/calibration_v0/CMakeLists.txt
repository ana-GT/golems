project(calibration_v0)

# Link directory
include_directories( ${CMAKE_SOURCE_DIR}/perception )
include_directories( ${CMAKE_SOURCE_DIR} )
link_directories( ${CMAKE_SOURCE_DIR}/lib )

# Find DART
list(APPEND CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_INSTALL_PREFIX}/share/dart")
find_package(DART REQUIRED)
include_directories(${DART_INCLUDE_DIRS})
link_directories(${DART_LIBRARY_DIRS})

# PCL
find_package( PCL COMPONENTS common search )
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

# Alvar
find_package( alvar )
include_directories( ${ALVAR_INCLUDE_DIRS} )

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Add aoi
set( GM_DIR $ENV{HOME}/local/share )
find_package(GM REQUIRED)
include_directories( ${GM_INCLUDE_DIRS} )
link_directories( ${GM_LIBRARY_DIRS} )

# Calibration v0
add_executable( calibration_v0 calibration_v0.cpp ) 
target_link_libraries( calibration_v0 utils aoi_utils motionControl bimanual_msgs crichton_markerDetector calibration ${OpenCV_LIBRARIES} ${ALVAR_LIBRARIES} ${DART_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_SEARCH_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} ) 
set_target_properties( calibration_v0 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin 
ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ) 

# Calibration v1
add_executable( calibration_v1_alita calibration_v1_alita.cpp ) 
target_link_libraries( calibration_v1_alita utils aoi_utils motionControl bimanual_msgs crichton_markerDetector calibration ${OpenCV_LIBRARIES} ${ALVAR_LIBRARIES} ${DART_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_SEARCH_LIBRARIES} ${AMINO_LIBRARIES} ) 
set_target_properties( calibration_v1_alita PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin 
ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ) 

#add_library( test_alvar3Daccuracy test_alvar3Daccuracy.cpp ) # ${SRCS} )
#target_link_libraries( test_alvar3Daccuracy ${OpenCV_LIBRARIES} ${ALVAR_LIBRARIES}  ${QT_LIBRARIES}    ${DART_LIBRARIES}  ${GRIP_LIBRARIES} ) 
#set_target_properties( test_alvar3Daccuracy PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin 
#ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ) 

#add_executable( getMarkersRobotPose getMarkersRobotPose.cpp ) 
#target_link_libraries( getMarkersRobotPose aoi_utils ${OpenCV_LIBRARIES} ${ALVAR_LIBRARIES}  ${QT_LIBRARIES}    ${DART_LIBRARIES}${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} ${golems_LIBRARIES} ) 
#set_target_properties( getMarkersRobotPose PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin 
#ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ) 

# Set project libraries variable
set(project_libs calibration_v0 )
