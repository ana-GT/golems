# Project sq_fitting
project( sq_fitting )

set( CMAKE_CXX_FLAGS "-std=gnu++0x -fPIC" )
include_directories( ${PROJECT_SOURCE_DIR}/include )
include_directories( ${PROJECT_SOURCE_DIR} )

# levmar
add_subdirectory( levmar )

# PCL
find_package( PCL )
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

# SQ_utils library
add_library( sq_fitting SQ_utils.cpp analytic_equations.cpp SQ_fitter.cpp )
target_link_libraries( sq_fitting levmar  ${PCL_LIBRARIES}  pthread )
set_target_properties( sq_fitting PROPERTIES LIBRARY_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/lib ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

# INSTALL
install( DIRECTORY ./ DESTINATION include/golems/sq_fitting FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" )
install( TARGETS sq_fitting DESTINATION lib )


# Add tests
add_subdirectory( tests )


