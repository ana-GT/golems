project( tests_basics )

find_package( PCL )
include_directories( ${PCL_INCLUDE_DIRS} )

add_executable( bending_deformations bending_deformations.cpp ) 
target_link_libraries( bending_deformations sq_utils sq_fitting )
set_target_properties( bending_deformations PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )
