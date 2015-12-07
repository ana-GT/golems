project( semantic_grasping )

find_package( PCL REQUIRED )
include_directories( ${PCL_INCLUDE_DIRS} ) 


add_executable( basic_kMeans basic_kMeans.cpp kmeans.cpp )
target_link_libraries( basic_kMeans ${PCL_LIBRARIES} )
set_target_properties( basic_kMeans PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )