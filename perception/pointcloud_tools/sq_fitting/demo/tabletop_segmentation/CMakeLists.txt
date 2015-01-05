# tabletop_segmentation
project( tabletop_segmentation)

# Plane segmentation
add_library( tabletop_segmentation tabletop_segmentation.cpp )
target_link_libraries( tabletop_segmentation ${PCL_LIBRARIES} )
set_target_properties( tabletop_segmentation PROPERTIES 
		       ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
		       RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

