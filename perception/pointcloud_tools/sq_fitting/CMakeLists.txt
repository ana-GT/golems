# Project sq_fitting
project( sq_fitting )

set( CMAKE_CXX_FLAGS "-std=gnu++0x -fPIC" )
include_directories( ${PROJECT_SOURCE_DIR}/include )
include_directories( ${PROJECT_SOURCE_DIR} )

# levmar
add_subdirectory( levmar )

# PCL
find_package( PCL COMPONENTS common io filters)
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

# SQ_utils library
add_library( sq_utils SQ_utils.cpp )
target_link_libraries( sq_utils  ${PCL_COMMON_LIBRARIES} )
set_target_properties( sq_utils PROPERTIES LIBRARY_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/lib ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )


add_library( sq_fitting SQ_utils.cpp analytic_equations.cpp SQ_fitter.cpp )
target_link_libraries( sq_fitting levmar  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}  pthread )
set_target_properties( sq_fitting PROPERTIES LIBRARY_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/lib ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

add_library( sq_deformations SQ_deformations.cpp )
target_link_libraries( sq_deformations  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}  pthread )
set_target_properties( sq_deformations PROPERTIES LIBRARY_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/lib ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

# Trials
add_library( evaluated_eqs evaluated_eqs.cpp )
target_link_libraries( evaluated_eqs levmar ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}  pthread )
set_target_properties( evaluated_eqs PROPERTIES LIBRARY_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/lib ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

add_executable( test_evaluated test_evaluated.cpp )
target_link_libraries( test_evaluated evaluated_eqs sq_fitting levmar ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}  pthread )
set_target_properties( test_evaluated PROPERTIES RUNTIME_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )



# INSTALL
install( DIRECTORY ./ DESTINATION include/golems/sq_fitting FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" )
install( TARGETS sq_utils DESTINATION lib )
install( TARGETS sq_fitting DESTINATION lib )


# Add tests
add_subdirectory( tests )


