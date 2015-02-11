# tabletop_symmetry: From paper by Bohg et al, 2010
project( refresher_utils )
set( CMAKE_CXX_FLAGS "-fPIC" )

include_directories( ${PROJECT_SOURCE_DIR} )

# PCL
find_package( PCL REQUIRED )
include_directories( ${PCL_INCLUDE_DIRS} )

# DART
find_package( DART REQUIRED )
include_directories( ${DART_INCLUDE_DIRS} )

# Symmetry
add_library( refresher_utils Refresher_utils.cpp )
target_link_libraries( refresher_utils  ${DART_LIBRARIES} ${PCL_LIBRARIES} )
set_target_properties( refresher_utils PROPERTIES 
		       ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
		       RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

# INSTALL
install( DIRECTORY ./ DESTINATION include/golems/refresher_utils FILES_MATCHING PATTERN "*.h" )
install( TARGETS refresher_utils DESTINATION lib )
