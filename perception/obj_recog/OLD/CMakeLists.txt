project( Obj_Recog )

# PCL
find_package( PCL )
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

# Utils
#add_library( obj_recog_utils obj_recog_utils.cpp )
#target_link_libraries( obj_recog_utils ${PCL_LIBRARIES} ) 
#set_target_properties( obj_recog_utils PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

add_subdirectory( iccv2011 )