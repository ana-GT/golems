# tabletop_symmetry: From paper by Bohg et al, 2010
project( tabletop_symmetry )

find_package( Qt5Widgets REQUIRED )

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Symmetry
add_library( mindGapper SHARED mindGapper.cpp dt/dt.cpp )
target_link_libraries( mindGapper ${OpenCV_LIBRARIES} ${PCL_COMMON_LIBRARIES}  )
set_target_properties( mindGapper PROPERTIES 
		       LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
		       RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

# INSTALL
install( DIRECTORY ./ DESTINATION include/golems/perception/pointcloud_tools/tabletop_symmetry FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" )
install( TARGETS mindGapper DESTINATION lib )
