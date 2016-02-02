# tabletop_segmentation
project( tabletop_segmentation)

# PCL
find_package( PCL COMPONENTS common search )
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

# Plane segmentation
add_library( tabletop_segmentation SHARED tabletop_segmentation.cpp )
target_link_libraries( tabletop_segmentation ${PCL_COMMON_LIBRARIES} ${PCL_SEARCH_LIBRARIES} )
set_target_properties( tabletop_segmentation PROPERTIES 
		       ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
		       LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

# Install
install( TARGETS tabletop_segmentation DESTINATION lib )
