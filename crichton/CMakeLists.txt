project( crichton )

add_subdirectory( motion_control )
add_subdirectory( tests )
#add_subdirectory( executables )


install( DIRECTORY  motion_control  DESTINATION include/golems/motion_control FILES_MATCHING PATTERN "*.h" )
