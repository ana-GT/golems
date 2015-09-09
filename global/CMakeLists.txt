project( global )

set( CMAKE_CXX_FLAGS "-fPIC" )

# Include
add_library( global crichton_global.cpp )

# Install
install( TARGETS global DESTINATION lib )
install( DIRECTORY  ./  DESTINATION include/golems/global FILES_MATCHING PATTERN "*.h" )
