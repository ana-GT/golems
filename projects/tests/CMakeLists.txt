project( tests_basics )

find_package( PCL COMPONENTS common io filters surface io features )
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )

add_executable( bending_deformations bending_deformations.cpp ) 
target_link_libraries( bending_deformations sq_utils sq_fitting  ${PCL_COMMON_LIBRARIES} ${PCL_SURFACE_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FEATURES_LIBRARIES} )
set_target_properties( bending_deformations PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )
