# Messages
project( bimanual_msgs )


add_library( bimanual_msgs SHARED bimanual_msgs.cpp )
target_link_libraries( bimanual_msgs  ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} lapack blas gfortran )
set_target_properties( bimanual_msgs PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

# Install
install( TARGETS bimanual_msgs DESTINATION lib )