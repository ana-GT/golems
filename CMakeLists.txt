cmake_minimum_required( VERSION 2.8 )

set( CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake/Modules" )


# Subdirectories
add_subdirectory( perception )
add_subdirectory( crichton )