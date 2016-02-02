project( global )

set( CMAKE_CXX_FLAGS "-fPIC" )

# Include
add_library( global SHARED crichton_global.cpp )
set_target_properties( global PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

# Install
install( TARGETS global DESTINATION lib )
install( DIRECTORY  ./  DESTINATION include/golems/global FILES_MATCHING PATTERN "*.h" )
