##############################
# Run test
##############################
include_directories( ${CMAKE_SOURCE_DIR} )

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( test_mirror test_mirror.cpp )
target_link_libraries( test_mirror SQ_fitter mindGapper ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBRARIES} pthread )
set_target_properties( test_mirror PROPERTIES RUNTIME_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


add_executable( SQ_fitter_test1 SQ_fitter_test1.cpp )
target_link_libraries( SQ_fitter_test1 SQ_fitter ${PCL_LIBRARIES} ${CERES_LIBRARIES} pthread )
set_target_properties( SQ_fitter_test1 PROPERTIES RUNTIME_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( cloud_basicOperations cloud_basicOperations.cpp )
target_link_libraries( cloud_basicOperations SQ_fitter ${PCL_LIBRARIES} )
set_target_properties( cloud_basicOperations PROPERTIES RUNTIME_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( SQ_utils_SE_test SQ_utils_SE_test.cpp )
target_link_libraries( SQ_utils_SE_test SQ_utils ${PCL_LIBRARIES} )
set_target_properties( SQ_utils_SE_test PROPERTIES RUNTIME_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


add_executable( sampleSQ sampleSQ.cpp )
target_link_libraries( sampleSQ SQ_utils ${PCL_LIBRARIES} )
set_target_properties( sampleSQ PROPERTIES RUNTIME_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( sampleSQ_naive sampleSQ_naive.cpp )
target_link_libraries( sampleSQ_naive SQ_utils ${PCL_LIBRARIES} )
set_target_properties( sampleSQ_naive PROPERTIES RUNTIME_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


add_executable( alterCloud alterCloud.cpp )
target_link_libraries( alterCloud ${PCL_LIBRARIES} )
set_target_properties( alterCloud PROPERTIES RUNTIME_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( levmar_test levmar_test.cpp )
target_link_libraries( levmar_test levmar lapack blas )
set_target_properties( levmar_test PROPERTIES RUNTIME_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( levmar_test5 levmar_test5.cpp )
target_link_libraries( levmar_test5 SQ_utils levmar lapack blas )
set_target_properties( levmar_test5 PROPERTIES RUNTIME_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

