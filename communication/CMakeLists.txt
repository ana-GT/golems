
project( communication )

include_directories( ${CMAKE_SOURCE_DIR} )
include_directories( . ) 

add_subdirectory( msgs )
add_subdirectory( test1_server )
add_subdirectory( test2_server )

#Install
install( DIRECTORY msgs DESTINATION include/golems/communication/ FILES_MATCHING PATTERN "*.h" )

