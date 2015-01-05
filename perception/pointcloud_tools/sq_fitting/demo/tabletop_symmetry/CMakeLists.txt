# tabletop_symmetry
project( tabletop_symmetry )

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# dt
add_subdirectory(dt)

# Symmetry
add_library( mindGapper mindGapper.cpp )
target_link_libraries( mindGapper dt ${OpenCV_LIBS} ${PCL_LIBRARIES} )
set_target_properties( mindGapper PROPERTIES 
		       ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
		       RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )
