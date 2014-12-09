#********************
# tests
#********************

include_directories( ${CMAKE_SOURCE_DIR}/crichton )
link_directories( ${CMAKE_SOURCE_DIR}/lib )

find_package( PkgConfig )
pkg_check_modules( EIGEN REQUIRED eigen3 )
include_directories( ${EIGEN_INCLUDE_DIRS} )


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

add_executable( moveLeftJoint moveLeftJoint.cpp )
target_link_libraries( moveLeftJoint ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} rt blas lapack)
set_target_properties( moveLeftJoint PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( moveLeftArm_loop moveLeftArm_loop.cpp )
target_link_libraries( moveLeftArm_loop ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} rt blas lapack)
set_target_properties( moveLeftArm_loop PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


add_executable( read_leftArm_state read_leftArm_state.cpp )
target_link_libraries( read_leftArm_state ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} rt blas lapack)
set_target_properties( read_leftArm_state PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


add_executable( read_rightArm_state read_rightArm_state.cpp )
target_link_libraries( read_rightArm_state ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} rt blas lapack)
set_target_properties( read_rightArm_state PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( basicControl_test1 basicControl_test1.cpp )
target_link_libraries( basicControl_test1  basicControl ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} rt blas lapack)
set_target_properties( basicControl_test1 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


add_executable( basicControl_test2 basicControl_test2.cpp )
target_link_libraries( basicControl_test2  basicControl ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} rt blas lapack)
set_target_properties( basicControl_test2 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


add_executable( basicControl_right basicControl_right.cpp )
target_link_libraries( basicControl_right  basicControl ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} rt blas lapack)
set_target_properties( basicControl_right PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


add_executable( test_timestamp testTimestamp.cpp )
target_link_libraries( test_timestamp  ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} rt blas lapack)
set_target_properties( test_timestamp PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


add_executable( monotonic monotonic.cpp )
target_link_libraries( monotonic  ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} rt blas lapack)
set_target_properties( monotonic PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( joint_vel_test1 joint_vel_test1.cpp )
target_link_libraries( joint_vel_test1  ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} rt blas lapack)
set_target_properties( joint_vel_test1 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( joint_vel_test2 joint_vel_test2.cpp )
target_link_libraries( joint_vel_test2  tkunz basicControl ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} rt blas lapack)
set_target_properties( joint_vel_test2 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( traj_test1 traj_test1.cpp )
target_link_libraries( traj_test1  tkunz basicControl ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} rt blas lapack)
set_target_properties( traj_test1 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( traj_test_receiver traj_test_receiver.cpp )
target_link_libraries( traj_test_receiver  tkunz basicControl ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} rt blas lapack)
set_target_properties( traj_test_receiver PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


add_executable( ach_send ach_send.cpp )
target_link_libraries( ach_send   ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} rt blas lapack)
set_target_properties( ach_send PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( ach_receive ach_receive.cpp )
target_link_libraries( ach_receive  ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} rt blas lapack)
set_target_properties( ach_receive PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


add_executable( test_motionControl test_motionControl.cpp )
target_link_libraries( test_motionControl motionControl ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} rt blas lapack)
set_target_properties( test_motionControl PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

