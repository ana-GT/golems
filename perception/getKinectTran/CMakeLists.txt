#****************************************
# Project kinect extrinsic calibration
#****************************************
project( getKinectTran )

# CMake module path (for ACH, Amino and SNS)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules" )

# Eigen
include_directories( /usr/include/eigen3 )
include_directories( ${PROJECT_SOURCE_DIR} )

# OpenCV
find_package( OpenCV )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Add LWA4 software paths
# amino, ach and SNS
find_package( amino REQUIRED )
include_directories( ${AMINO_INCLUDE_DIRS} )
find_package( ach REQUIRED )
include_directories( ${ACH_INCLUDE_DIRS} )
find_package( sns REQUIRED )
include_directories( ${SNS_INCLUDE_DIRS} )

# PCL
find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# Add levmar
add_subdirectory( levmar )

# Add project
#add_executable( getKinectTran getKinectTran.cpp )
#target_link_libraries( getKinectTran ${OpenCV_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} ${PCL_LIBRARIES} lapack blas gfortran )
#set_target_properties( getKinectTran PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
#  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

#add_executable( moveEE_predef moveEE_predef.cpp )
#target_link_libraries( moveEE_predef ${DART_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} lapack blas gfortran rt )
#set_target_properties( moveEE_predef PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
#  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


#add_executable( moveEE_automatic moveEE_automatic.cpp )
#target_link_libraries( moveEE_automatic ${DART_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} rt lapack blas gfortran )
#set_target_properties( moveEE_automatic PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
#  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


# Apps
#add_executable( pickObject pickObject.cpp )
#target_link_libraries( pickObject ${OpenCV_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} lapack blas gfortran )
#set_target_properties( pickObject PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
#  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


# Tests
include_directories( ${CMAKE_SOURCE_DIR} )

add_executable( testMethods testMethods.cpp levmar_eqs.cpp )
target_link_libraries( testMethods levmar ${PCL_LIBRARIES} lapack blas gfortran )
set_target_properties( testMethods PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )
