project(correspondence_grouping)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories( ${CMAKE_SOURCE_DIR}/perception )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/jsoncpp )

add_executable (global_recognition global_recognition.cpp GlobalRecognizer.cpp jsoncpp/jsoncpp.cpp )
target_link_libraries (global_recognition tabletop_segmentation ${PCL_LIBRARIES})
set_target_properties( global_recognition PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )
