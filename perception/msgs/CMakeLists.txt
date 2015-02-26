# Messages
project( perception_msgs )

add_library( perception_msgs perception_msgs.cpp )
target_link_libraries( perception_msgs  ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} lapack blas gfortran )
set_target_properties( perception_msgs PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

# Install
install( TARGETS perception_msgs DESTINATION lib )
