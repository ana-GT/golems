# tabletop_symmetry: From paper by Bohg et al, 2010
project( refresher_utils )

set(CMAKE_INSTALL_PREFIX /usr/local )
set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-fPIC -std=c++11" )

# PCL
find_package( PCL REQUIRED COMPONENTS common surface filters )
include_directories( ${PCL_INCLUDE_DIRS} )

# Symmetry
add_library( refresher_utils STATIC Refresher_utils.cpp )
target_link_libraries( refresher_utils  ${PCL_COMMON_LIBRARIES} ${PCL_SURFACE_LIBRARIES} ${PCL_FILTERS_LIBRARIES} )
set_target_properties( refresher_utils PROPERTIES 
		       ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
		       RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

# INSTALL
install( DIRECTORY ./ DESTINATION include/golems/perception/pointcloud_tools/refresher_utils FILES_MATCHING PATTERN "*.h" )
install( TARGETS refresher_utils DESTINATION lib )
