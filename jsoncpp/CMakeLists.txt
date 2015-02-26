project( jsoncpp )

add_library( jsoncpp jsoncpp.cpp )
#target_link_libraries( jsoncpp ${PCL_LIBRARIES} )
set_target_properties( jsoncpp PROPERTIES 
		       ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
		       RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )
