cmake_minimum_required( VERSION 2.8 )

add_executable( test_icp test_icp.cpp )
target_link_libraries( test_icp ${PCL_LIBRARIES} )
set_target_properties( test_icp PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( test_icp_2 test_icp_2.cpp )
target_link_libraries( test_icp_2 ${PCL_LIBRARIES} )
set_target_properties( test_icp_2 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )
