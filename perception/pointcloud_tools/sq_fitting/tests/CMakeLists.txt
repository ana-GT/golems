##############################
# Run test
##############################
include_directories( ${CMAKE_SOURCE_DIR} )

link_directories( ${PCL_LIBRARY_DIRS} )
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}" )

# OpenCV
#find_package( OpenCV REQUIRED )
#include_directories( ${OpenCV_INCLUDE_DIRS} )

#add_executable( test_mirror test_mirror.cpp )
#target_link_libraries( test_mirror SQ_fitter mindGapper ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBRARIES} pthread )
#set_target_properties( test_mirror PROPERTIES RUNTIME_OUTPUT_DIRECTORY
#  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( sampleSQ sampleSQ.cpp )
target_link_libraries( sampleSQ sq_utils ${PCL_LIBRARIES} )
set_target_properties( sampleSQ PROPERTIES RUNTIME_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


add_executable( es_tests_0 es_tests_0.cpp )
target_link_libraries( es_tests_0 evaluated_eqs sq_fitting ${PCL_LIBRARIES} )
set_target_properties( es_tests_0 PROPERTIES RUNTIME_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( es_tests_1 es_tests_1.cpp )
target_link_libraries( es_tests_1 evaluated_eqs sq_fitting ${PCL_LIBRARIES} levmar )
set_target_properties( es_tests_1 PROPERTIES RUNTIME_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( es_read_tests_1 es_read_tests_1.cpp )
target_link_libraries( es_read_tests_1 evaluated_eqs sq_fitting ${PCL_LIBRARIES} levmar )
set_target_properties( es_read_tests_1 PROPERTIES RUNTIME_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( es_tests_2 es_tests_2.cpp )
target_link_libraries( es_tests_2 evaluated_eqs sq_fitting ${PCL_LIBRARIES} levmar )
set_target_properties( es_tests_2 PROPERTIES RUNTIME_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( es_read_tests_2 es_read_tests_2.cpp )
target_link_libraries( es_read_tests_2 evaluated_eqs sq_fitting ${PCL_LIBRARIES} levmar )
set_target_properties( es_read_tests_2 PROPERTIES RUNTIME_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


#add_executable( es_tests_3 es_tests_3.cpp )
#target_link_libraries( es_tests_3 evaluated_eqs sq_fitting ${PCL_LIBRARIES} levmar )
#set_target_properties( es_tests_3 PROPERTIES RUNTIME_OUTPUT_DIRECTORY
#  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


#add_executable( es_tests_4 es_tests_4.cpp )
#target_link_libraries( es_tests_4 evaluated_eqs sq_fitting ${PCL_LIBRARIES} levmar )
#set_target_properties( es_tests_4 PROPERTIES RUNTIME_OUTPUT_DIRECTORY
#  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( es_tests_5 es_tests_5.cpp )
target_link_libraries( es_tests_5 evaluated_eqs sq_fitting ${PCL_LIBRARIES} levmar )
set_target_properties( es_tests_5 PROPERTIES RUNTIME_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( es_tests_6 es_tests_6.cpp )
target_link_libraries( es_tests_6 evaluated_eqs sq_fitting ${PCL_LIBRARIES} levmar )
set_target_properties( es_tests_6 PROPERTIES RUNTIME_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )



#add_executable( SQ_fitter_test1 SQ_fitter_test1.cpp )
#target_link_libraries( SQ_fitter_test1 sq_fitting ${PCL_COMMON_LIBRARIES} ${PCL_FILTERS_LIBRARIES}  pthread )
#set_target_properties( SQ_fitter_test1 PROPERTIES RUNTIME_OUTPUT_DIRECTORY
#  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


#add_executable( sq_deformations_test sq_deformations_test.cpp )
#target_link_libraries( sq_deformations_test sq_fitting sq_deformations ${PCL_LIBRARIES} )
#set_target_properties( sq_deformations_test PROPERTIES RUNTIME_OUTPUT_DIRECTORY
#  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

#add_executable( sq_deformations_test_noise sq_deformations_test_noise.cpp )
#target_link_libraries( sq_deformations_test_noise sq_fitting sq_deformations ${PCL_LIBRARIES} )
#set_target_properties( sq_deformations_test_noise PROPERTIES RUNTIME_OUTPUT_DIRECTORY
#  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


#add_executable( test_cases_generation test_cases_generation.cpp )
#target_link_libraries( test_cases_generation evaluated_eqs sq_fitting ${PCL_LIBRARIES} )
#set_target_properties( test_cases_generation PROPERTIES RUNTIME_OUTPUT_DIRECTORY
#  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


#add_executable( deformations_tests_0 deformations_tests_0.cpp )
#target_link_libraries( deformations_tests_0 sq_deformations sq_utils ${PCL_LIBRARIES} )
#set_target_properties( deformations_tests_0 PROPERTIES RUNTIME_OUTPUT_DIRECTORY
#  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

#add_executable( deformations_tests_1 deformations_tests_1.cpp )
#target_link_libraries( deformations_tests_1 sq_deformations sq_utils ${PCL_LIBRARIES} )
#set_target_properties( deformations_tests_1 PROPERTIES RUNTIME_OUTPUT_DIRECTORY
#  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


#add_executable( read_data_SQ_function read_data_SQ_function.cpp )
#set_target_properties( read_data_SQ_function PROPERTIES RUNTIME_OUTPUT_DIRECTORY
#  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )





