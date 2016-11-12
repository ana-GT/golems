cmake_minimum_required( VERSION 2.8 )
project(theremin)

#-----------------------------------------------
#                COMPILE SETTINGS
#-----------------------------------------------
set(CMAKE_BUILD_TYPE "Debug")
set(BUILD_FLAGS "-std=gnu++0x -Wreturn-type -Wno-enum-compare -Wall")
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/build)
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_MODULE_PATH  ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

#-----------------------------------------------
#                FIND PACKAGES
#-----------------------------------------------
# Find QT
find_package(Qt5Gui REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5Xml REQUIRED)
find_package(Qt5Widgets REQUIRED )
find_package(Qt5Bluetooth REQUIRED ) 

add_definitions(${QT_DEFINITIONS})
ADD_DEFINITIONS(-DQT_PLUGIN)
ADD_DEFINITIONS(-DQT_NO_DEBUG)
ADD_DEFINITIONS(-DQT_SHARED)

#-----------------------------------------------
#      INCLUDES & SUBDIRECTORY LIBRARIRES
#-----------------------------------------------
# Include top-level include directory
include_directories(app/include)
include_directories(${EXECUTABLE_OUTPUT_PATH})

# Add include directory in order to find ui_*.h files
include_directories(${CMAKE_BINARY_DIR}/theremin)

# Wrap .ui files
FILE(GLOB UI_FILES ./app/ui/*.ui)
QT5_WRAP_UI(UI ${UI_FILES})

# Wrap .moc files
FILE(GLOB MOC_FILES ./app/include/*.h)
QT5_WRAP_CPP(MOC ${MOC_FILES})

# Add .qrc files
file(GLOB RC_FILES .rc/*.qrc)
QT5_ADD_RESOURCES(RESOURCES ${RC_FILES})

# Add source and header files
#file(GLOB srcs ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp ${CMAKE_CURRENT_LIST_DIR}/include/*.h)
file(GLOB SRCS app/include/*.h app/src/main.cpp app/src/theremin.cpp )


# Run tinyalsa
include_directories( tinyalsa/include app/include )
link_directories( ${CMAKE_SOURCE_DIR}/lib )

add_library( tinyalsa SHARED tinyalsa/pcm.c tinyalsa/mixer.c )
#target_link_libraries( tinyalsa asound)
set_target_properties( tinyalsa PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib 
ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib ) 

#add_executable( tinyplay tinyalsa/tinyplay.c )
#target_link_libraries( tinyplay tinyalsa )
#set_target_properties( tinyplay PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin 
#ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ) 



# Add Qt Widgets library
add_executable( theremin ${SRCS} ${UI} ${MOC} ${RESOURCES})
target_link_libraries( theremin  Qt5::Widgets Qt5::Gui Qt5::Core Qt5::OpenGL Qt5::Xml tinyalsa asound )
set_target_properties( theremin PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin 
ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ) 

#add_executable( test src/test.cpp )
#target_link_libraries( test asound )
#set_target_properties( test PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin 
#ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ) 

#add_executable( wf src/wf.cpp )
#target_link_libraries( wf asound )
#set_target_properties( wf PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin 
#ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ) 

#add_executable( testunits src/testunits.cpp )
#target_link_libraries( testunits asound )
#set_target_properties( testunits PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin 
#ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ) 
