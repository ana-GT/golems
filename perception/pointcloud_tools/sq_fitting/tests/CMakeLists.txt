##############################
# Run test
##############################
include_directories( ${CMAKE_SOURCE_DIR} )

link_directories( ${PCL_LIBRARY_DIRS} )

# OpenCV
#find_package( OpenCV REQUIRED )
#include_directories( ${OpenCV_INCLUDE_DIRS} )

#add_executable( test_mirror test_mirror.cpp )
#target_link_libraries( test_mirror SQ_fitter mindGapper ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBRARIES} pthread )
#set_target_properties( test_mirror PROPERTIES RUNTIME_OUTPUT_DIRECTORY
#  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


add_executable( SQ_fitter_test1 SQ_fitter_test1.cpp )
target_link_libraries( SQ_fitter_test1 sq_fitting ${PCL_COMMON_LIBRARIES} ${PCL_FILTERS_LIBRARIES}  pthread )
set_target_properties( SQ_fitter_test1 PROPERTIES RUNTIME_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


add_executable( sq_deformations_test sq_deformations_test.cpp )
target_link_libraries( sq_deformations_test sq_fitting sq_deformations ${PCL_LIBRARIES} )
set_target_properties( sq_deformations_test PROPERTIES RUNTIME_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( sq_deformations_test_noise sq_deformations_test_noise.cpp )
target_link_libraries( sq_deformations_test_noise sq_fitting sq_deformations ${PCL_LIBRARIES} )
set_target_properties( sq_deformations_test_noise PROPERTIES RUNTIME_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( test_cases_generation test_cases_generation.cpp )
target_link_libraries( test_cases_generation evaluated_eqs sq_fitting ${PCL_LIBRARIES} )
set_target_properties( test_cases_generation PROPERTIES RUNTIME_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( test_cases_generation_short test_cases_generation_short.cpp )
target_link_libraries( test_cases_generation_short evaluated_eqs sq_fitting ${PCL_LIBRARIES} )
set_target_properties( test_cases_generation_short PROPERTIES RUNTIME_OUTPUT_DIRECTORY
  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )



#add_executable( sampleSQ_naive sampleSQ_naive.cpp )
#target_link_libraries( sampleSQ_naive SQ_utils ${PCL_LIBRARIES} )
#set_target_properties( sampleSQ_naive PROPERTIES RUNTIME_OUTPUT_DIRECTORY
#  ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


