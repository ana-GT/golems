project( marker_detection )

include_directories( ${CMAKE_SOURCE_DIR}/perception )

# OpenCV
set( OpenCV_DIR /home/ana/local/share/OpenCV )
set( OpenCV_CONFIG_PATH /home/ana/local/share/OpenCV )
find_package( OpenCV 2.4.8 EXACT REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Alvar
find_package( alvar )
include_directories( ${ALVAR_INCLUDE_DIRS} )



add_executable( markerDetection marker_detection.cpp crichton_markerDetector.cpp )
target_link_libraries( markerDetection  ${OpenCV_LIBRARIES} ${ALVAR_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} )
set_target_properties( markerDetection PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )
