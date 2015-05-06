# Messages
project( server_msgs )

add_library( server_msgs server_msgs.cpp )
target_link_libraries( server_msgs  ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} lapack blas gfortran )
set_target_properties( server_msgs PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

# Install
install( TARGETS server_msgs DESTINATION lib )
