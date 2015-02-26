#*********************************
# executables to run on calvin
#*********************************

include_directories( ${CMAKE_SOURCE_DIR}/crichton )
link_directories( ${CMAKE_SOURCE_DIR}/lib )

find_package( PkgConfig )
pkg_check_modules( EIGEN REQUIRED eigen3 )
include_directories( ${EIGEN_INCLUDE_DIRS} )




add_executable( crichton_trajGen crichton_trajGen.cpp )
target_link_libraries( crichton_trajGen basicControl  ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} rt blas lapack)
set_target_properties( crichton_trajGen PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

