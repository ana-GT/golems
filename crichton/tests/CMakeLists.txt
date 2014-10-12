#********************
# tests
#********************

add_executable( readHandStates readHandStates.cpp )
target_link_libraries( readHandStates ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} rt blas lapack)
set_target_properties( readHandStates PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


add_executable( moveFingers moveFingers.cpp )
target_link_libraries( moveFingers ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} rt blas lapack)
set_target_properties( moveFingers PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( readArmStates readArmStates.cpp )
target_link_libraries( readArmStates ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} rt blas lapack)
set_target_properties( readArmStates PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( moveArmBasic moveArmBasic.cpp )
target_link_libraries( moveArmBasic ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} rt blas lapack)
set_target_properties( moveArmBasic PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )
