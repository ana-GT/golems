project(client_pick)

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
include_directories(include)
include_directories(${EXECUTABLE_OUTPUT_PATH})

# Add include directory in order to find ui_*.h files
include_directories(${CMAKE_BINARY_DIR}/communication/client_pick)

# Wrap .ui files
FILE(GLOB UI_FILES ./ui/*.ui)
QT5_WRAP_UI(UI ${UI_FILES})

# Wrap .moc files
FILE(GLOB MOC_FILES ./include/*.h)
QT5_WRAP_CPP(MOC ${MOC_FILES})

# Add .qrc files
file(GLOB RC_FILES .rc/*.qrc)
QT5_ADD_RESOURCES(RESOURCES ${RC_FILES})

# Add source and header files
#file(GLOB srcs ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp ${CMAKE_CURRENT_LIST_DIR}/include/*.h)
file(GLOB SRCS include/*.h src/*.cpp)

# Add LWA4 software paths / include / libraries
# amino, ach and sns
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules" )


find_package( amino REQUIRED )
include_directories( ${AMINO_INCLUDE_DIRS} )
find_package( ach REQUIRED )
include_directories( ${ACH_INCLUDE_DIRS} )
find_package( sns REQUIRED )
include_directories( ${SNS_INCLUDE_DIRS} )

# Add motionControl
set( golems_DIR $ENV{HOME}/local/share )
find_package(golems REQUIRED)
include_directories( ${golems_INCLUDE_DIRS} )
link_directories( ${golems_LIBRARY_DIRS} )

# Add Qt Widgets library
add_executable( client_pick ${SRCS} ${UI} ${MOC} ${RESOURCES})
target_link_libraries( client_pick global Qt5::Widgets Qt5::Gui Qt5::Core Qt5::OpenGL Qt5::Xml Qt5::Bluetooth ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} )
set_target_properties( client_pick PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin 
ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ) 

