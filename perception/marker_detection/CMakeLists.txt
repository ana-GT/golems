project( marker_detection )

set(CMAKE_INSTALL_PREFIX /usr/local )
set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=gnu++11 -fPIC" )

# Alvar
find_package( alvar REQUIRED )
include_directories( ${ALVAR_INCLUDE_DIRS} )
message( STATUS "Let's see: OpenCV LIBRARIES: ${OpenCV_LIBRARIES}" )
message( STATUS "Let's see 2: ALVAR LIBRARIES: ${ALVAR_LIBRARIES} ")

# crichton_markerDetector
add_library( crichton_markerDetector SHARED crichton_markerDetector.cpp )
target_link_libraries( crichton_markerDetector  ${OpenCV_LIBRARIES} ${ALVAR_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} )
set_target_properties( crichton_markerDetector PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

# Install
install( TARGETS crichton_markerDetector DESTINATION lib )



