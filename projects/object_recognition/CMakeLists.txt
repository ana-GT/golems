#cmake_minimum_required( VERSION 2.8 )
project( OR )
#set( CMAKE_MODULE_PATH CMAKE_MODULE_PATH "/home/ana/local/share/Caffe/Caffe" )
#set( CMAKE_CXX_FLAGS   "-std=c++0x" )
set( CMAKE_CXX_FLAGS   "-std=gnu++11" )
find_package( Caffe REQUIRED )
include_directories( ${Caffe_INCLUDE_DIRS} )
add_definitions( ${Caffe_DEFINITIONS} )

find_package( OpenCV )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# PCL
find_package(PCL REQUIRED COMPONENTS common search visualization segmentation surface io )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Include
include_directories( ${CMAKE_SOURCE_DIR}/perception )
include_directories( ${CMAKE_SOURCE_DIR}/perception/pointcloud_tools )
include_directories( ${CMAKE_SOURCE_DIR} )

include_directories( ${CMAKE_SOURCE_DIR}/perception/pointcloud_tools/sq_fitting )

add_executable( recognition_3_speech_joystick recognition_3_speech_joystick.cpp classifier.cpp )
target_link_libraries( recognition_3_speech_joystick ${Caffe_LIBRARIES} ${OpenCV_LIBRARIES} tabletop_segmentation  ${PCL_SURFACE_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${ACH_LIBRARIES} ${SNS_LIBRARIES} )
set_target_properties( recognition_3_speech_joystick PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


add_executable( recognition_2_speech recognition_2_speech.cpp classifier.cpp )
target_link_libraries( recognition_2_speech ${Caffe_LIBRARIES} ${OpenCV_LIBRARIES} tabletop_segmentation  ${PCL_SURFACE_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${ACH_LIBRARIES} ${SNS_LIBRARIES} )
set_target_properties( recognition_2_speech PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( recognition_1 recognition_1.cpp classifier.cpp )
target_link_libraries( recognition_1 ${Caffe_LIBRARIES} ${OpenCV_LIBRARIES} tabletop_segmentation  ${PCL_SURFACE_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${ACH_LIBRARIES} ${SNS_LIBRARIES} )
set_target_properties( recognition_1 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


add_executable( recognition_0 recognition_0.cpp classifier.cpp )
target_link_libraries( recognition_0 ${Caffe_LIBRARIES} ${OpenCV_LIBRARIES} )
set_target_properties( recognition_0 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( features_0 features_0.cpp feature_extractor.cpp )
target_link_libraries( features_0 ${Caffe_LIBRARIES} ${OpenCV_LIBRARIES} )
set_target_properties( features_0 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( store_features store_features.cpp classifier.cpp )
target_link_libraries( store_features ${Caffe_LIBRARIES} ${OpenCV_LIBRARIES} )
set_target_properties( store_features PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( store_images_preprocessed store_images_preprocessed.cpp )
target_link_libraries( store_images_preprocessed ${Caffe_LIBRARIES} ${OpenCV_LIBRARIES} )
set_target_properties( store_images_preprocessed PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( see_images_preprocessed_db see_images_preprocessed_db.cpp )
target_link_libraries( see_images_preprocessed_db ${Caffe_LIBRARIES} ${OpenCV_LIBRARIES} )
set_target_properties( see_images_preprocessed_db PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( get_images_mean get_images_mean.cpp )
target_link_libraries( get_images_mean ${Caffe_LIBRARIES} ${OpenCV_LIBRARIES} )
set_target_properties( get_images_mean PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


add_executable( train_svm train_svm.cpp )
target_link_libraries( train_svm ${Caffe_LIBRARIES} ${OpenCV_LIBRARIES} )
set_target_properties( train_svm PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )
