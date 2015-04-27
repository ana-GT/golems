# tabletop_symmetry: From paper by Bohg et al, 2010
project( tabletop_symmetry )
set( CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS} -fPIC" )

include_directories( ${PROJECT_SOURCE_DIR} )

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Symmetry
add_library( mindGapper mindGapper.cpp dt/dt.cpp )
target_link_libraries( mindGapper ${OpenCV_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} )
set_target_properties( mindGapper PROPERTIES 
		       ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
		       RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

# INSTALL
install( DIRECTORY ./ DESTINATION include/golems/tabletop_symmetry FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" )
install( TARGETS mindGapper DESTINATION lib )
