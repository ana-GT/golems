# Project sq_fitting
project( sq_fitting )

set(CMAKE_INSTALL_PREFIX /usr/local )
set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=gnu++11 -fPIC" )

find_package( Qt5Widgets REQUIRED )

# levmar
add_subdirectory( levmar )

# PCL
find_package( PCL COMPONENTS common io filters surface io )
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

# SQ_utils library
add_library( sq_utils STATIC SQ_utils.cpp SQ_parameters.cpp )
target_link_libraries( sq_utils  ${PCL_COMMON_LIBRARIES} ${PCL_SURFACE_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${PCL_IO_LIBRARIES} )
set_target_properties( sq_utils PROPERTIES LIBRARY_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/lib ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )


add_library( sq_fitting STATIC  SQ_fitter.cpp SQ_fitter_t.cpp SQ_fitter_b.cpp  evaluated_eqs.cpp evaluated_eqs_t.cpp evaluated_eqs_b.cpp )
target_link_libraries( sq_fitting sq_utils levmar  ${PCL_COMMON_LIBRARIES} ${PCL_FILTERS_LIBRARIES}  pthread )
set_target_properties( sq_fitting PROPERTIES LIBRARY_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/lib ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

add_library( sq_deformations SQ_deformations.cpp )
target_link_libraries( sq_deformations  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}  pthread )
set_target_properties( sq_deformations PROPERTIES LIBRARY_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/lib ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

# Trials
add_library( evaluated_eqs evaluated_eqs.cpp evaluated_eqs_t.cpp evaluated_eqs_b.cpp  )
target_link_libraries( evaluated_eqs levmar ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}  pthread )
set_target_properties( evaluated_eqs PROPERTIES LIBRARY_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/lib ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )



# INSTALL
install( DIRECTORY ./ DESTINATION include/golems/perception/pointcloud_tools/sq_fitting FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" )
install( TARGETS sq_utils DESTINATION lib )
install( TARGETS sq_fitting DESTINATION lib )


# Add tests
#add_subdirectory( tests )


