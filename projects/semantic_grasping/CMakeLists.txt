project( semantic_grasping )

find_package( PCL REQUIRED )
include_directories( ${PCL_INCLUDE_DIRS} ) 


# Include
#include_directories( ${CMAKE_SOURCE_DIR}/perception )
#include_directories( ${CMAKE_SOURCE_DIR}/perception/pointcloud_tools )
#include_directories( ${CMAKE_SOURCE_DIR} )

#include_directories( ${CMAKE_SOURCE_DIR}/perception/pointcloud_tools/sq_fitting )


add_executable( basic_kMeans basic_kMeans.cpp kmeans.cpp )
target_link_libraries( basic_kMeans ${PCL_LIBRARIES} )
set_target_properties( basic_kMeans PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

#add_executable( basic_kMeans_2 basic_kMeans_2.cpp kmeans.cpp )
#target_link_libraries( basic_kMeans_2 sq_fitting ${PCL_LIBRARIES} )
#set_target_properties( basic_kMeans_2 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
#  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )
