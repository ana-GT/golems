
project( communication )
 

add_subdirectory( msgs )
add_subdirectory( test1_server )
add_subdirectory( test2_server )
#add_subdirectory( server_pap )
#add_subdirectory( client_pap )
add_subdirectory( server_pick )
add_subdirectory( client_pick )

#Install
install( DIRECTORY msgs DESTINATION include/golems/communication/ FILES_MATCHING PATTERN "*.h" )

