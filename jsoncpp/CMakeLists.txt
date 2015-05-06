project( jsoncpp )
set( CMAKE_CXX_FLAGS "-fPIC" )
add_library( jsoncpp jsoncpp.cpp )
set_target_properties( jsoncpp PROPERTIES 
		       ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
		       RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )



# Install
install( TARGETS jsoncpp DESTINATION lib )
install( DIRECTORY  ./  DESTINATION include/golems/jsoncpp FILES_MATCHING PATTERN "*.h" )
