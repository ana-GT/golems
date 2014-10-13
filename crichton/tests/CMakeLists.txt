#********************
# tests
#********************

message( STATUS "2 Amino include dir: ${AMINO_INCLUDE_DIRS}" )
message( STATUS "2 Amino lib dir: ${AMINO_LIBRARIES}" )
message( STATUS "2 SNS include dir: ${SNS_INCLUDE_DIRS}" )
message( STATUS "2 SNS lib dir: ${SNS_LIBRARIES}" )
message( STATUS "2 Ach include dir: ${ACH_INCLUDE_DIRS}" )
message( STATUS "2 Ach lib dir: ${ACH_LIBRARIES}" )


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


add_executable( moveLeftArm moveLeftArm.cpp )
target_link_libraries( moveLeftArm ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} rt blas lapack)
set_target_properties( moveLeftArm PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( moveLeftArm_loop moveLeftArm_loop.cpp )
target_link_libraries( moveLeftArm_loop ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} rt blas lapack)
set_target_properties( moveLeftArm_loop PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


add_executable( read_leftArm_state read_leftArm_state.cpp )
target_link_libraries( read_leftArm_state ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} rt blas lapack)
set_target_properties( read_leftArm_state PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


add_executable( read_rightArm_state read_rightArm_state.cpp )
target_link_libraries( read_rightArm_state ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} rt blas lapack)
set_target_properties( read_rightArm_state PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )
