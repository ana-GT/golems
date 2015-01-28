# Original source code from pcl/tutorials/iccv11
project(or)

#PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#json
include_directories( ${CMAKE_SOURCE_DIR}/jsoncpp )

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRECTORIES} )

#perception
include_directories( ${CMAKE_SOURCE_DIR}/perception )

# Object recognition
include_directories( ${CMAKE_SOURCE_DIR}/perception )

add_executable ( build_object_model build_object_model.cpp)
target_link_libraries ( build_object_model obj_recog jsoncpp  ${PCL_LIBRARIES} )
set_target_properties( build_object_model PROPERTIES 
		       ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
		       RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable ( build_all_object_models build_all_object_models.cpp )
target_link_libraries ( build_all_object_models obj_recog jsoncpp ${PCL_LIBRARIES} )
set_target_properties( build_all_object_models PROPERTIES 
		       ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
		       RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )
		     
add_executable ( objRecog_ui objRecog_ui.cpp)
target_link_libraries ( objRecog_ui obj_recog jsoncpp ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} )
set_target_properties( objRecog_ui PROPERTIES 
		       ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
		       RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )
		     
		     
#add_executable (iccv2011_capture_tool src/capture_tool.cpp src/openni2_capture.cpp)
#target_link_libraries (iccv2011_capture_tool ${PCL_LIBRARIES})

#add_executable (iccv2011_correspondence_viewer src/correspondence_viewer.cpp)
#target_link_libraries (iccv2011_correspondence_viewer ${PCL_LIBRARIES} obj_recog )

#add_executable (iccv2011_test_filters src/test_filters.cpp)
#target_link_libraries (iccv2011_test_filters ${PCL_LIBRARIES} obj_recog )

#add_executable (iccv2011_test_segmentation src/test_segmentation.cpp)
#target_link_libraries (iccv2011_test_segmentation ${PCL_LIBRARIES} obj_recog )

#add_executable (iccv2011_test_feature_estimation src/test_feature_estimation.cpp)
#target_link_libraries (iccv2011_test_feature_estimation ${PCL_LIBRARIES} obj_recog )

#add_executable (iccv2011_test_registration src/test_registration.cpp)
#target_link_libraries (iccv2011_test_registration ${PCL_LIBRARIES} obj_recog )

#add_executable (iccv2011_test_surface src/test_surface.cpp)
#target_link_libraries (iccv2011_test_surface ${PCL_LIBRARIES})

#add_executable (iccv2011_test_object_recognition src/test_object_recognition.cpp)
#target_link_libraries (iccv2011_test_object_recognition ${PCL_LIBRARIES} obj_recog )


#add_executable (iccv2011_tutorial src/tutorial.cpp)
#target_link_libraries (iccv2011_tutorial ${PCL_LIBRARIES} obj_recog )
