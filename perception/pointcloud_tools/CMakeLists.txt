# Project pointcloud_tools
project( pointcloud_tools )

include_directories( ${PROJECT_SOURCE_DIR} )

add_subdirectory( sq_fitting )
add_subdirectory( tabletop_symmetry )
add_subdirectory( refresher_utils )
add_subdirectory( base_object_recognition )
