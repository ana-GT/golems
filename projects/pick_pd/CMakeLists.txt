cmake_minimum_required( VERSION 2.8 )

# Find eigen
find_package( PkgConfig )
pkg_check_modules( EIGEN eigen3 )
include_directories( ${EIGEN_INCLUDE_DIRS} )

# Link to golems libraries
set( golems_DIR $ENV{HOME}/local/share )
find_package(golems REQUIRED)
include_directories( ${golems_INCLUDE_DIRS} )
link_directories( ${golems_LIBRARY_DIRS} )


add_library( pick_pd pick_pd.cpp )
target_link_libraries( pick_pd jsoncpp )
set_target_properties( pick_pd PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

add_executable( pick_pd_test pick_pd_test.cpp )
target_link_libraries( pick_pd_test pick_pd jsoncpp )
set_target_properties( pick_pd_test PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )