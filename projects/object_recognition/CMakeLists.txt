
project( object_recognition_utilities_demos )

set( CMAKE_CXX_FLAGS   "-std=gnu++11" )

# Caffe
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


include_directories( ${CMAKE_SOURCE_DIR}/perception/pointcloud_tools/sq_fitting )

add_executable( recognition_4_comparison recognition_4_comparison.cpp  )
target_link_libraries( recognition_4_comparison base_classifier ${Caffe_LIBRARIES} ${OpenCV_LIBRARIES} tabletop_segmentation  ${PCL_SURFACE_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${ACH_LIBRARIES} ${SNS_LIBRARIES} )
set_target_properties( recognition_4_comparison PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


add_executable( recognition_3_speech_joystick recognition_3_speech_joystick.cpp  )
target_link_libraries( recognition_3_speech_joystick objectsDatabase ${Caffe_LIBRARIES} ${OpenCV_LIBRARIES} tabletop_segmentation  ${PCL_SURFACE_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${ACH_LIBRARIES} ${SNS_LIBRARIES} )
set_target_properties( recognition_3_speech_joystick PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


add_executable( recognition_2_speech recognition_2_speech.cpp )
target_link_libraries( recognition_2_speech  objectsDatabase  ${Caffe_LIBRARIES} ${OpenCV_LIBRARIES} tabletop_segmentation  ${PCL_SURFACE_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${ACH_LIBRARIES} ${SNS_LIBRARIES} )
set_target_properties( recognition_2_speech PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( recognition_1 recognition_1.cpp  )
target_link_libraries( recognition_1  objectsDatabase  ${Caffe_LIBRARIES} ${OpenCV_LIBRARIES} tabletop_segmentation  ${PCL_SURFACE_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${ACH_LIBRARIES} ${SNS_LIBRARIES} )
set_target_properties( recognition_1 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


add_executable( recognition_0 recognition_0.cpp )
target_link_libraries( recognition_0 objectsDatabase ${Caffe_LIBRARIES} ${OpenCV_LIBRARIES} )
set_target_properties( recognition_0 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


add_executable( store_features store_features.cpp  )
target_link_libraries( store_features base_classifier ${Caffe_LIBRARIES} ${OpenCV_LIBRARIES} )
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
