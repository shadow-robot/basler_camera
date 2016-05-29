cmake_minimum_required(VERSION 2.8.3)
project(basler_camera)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    image_transport
    camera_info_manager
    cv_bridge
    dynamic_reconfigure
)
find_package(OpenCV REQUIRED)

find_path(pylon_BIN_PATH pylon-config
    PATHS /opt/pylon5 /usr/local /usr/local/pylon5 ENV PYLON_ROOT
    PATH_SUFFIXES bin)
execute_process(COMMAND ${pylon_BIN_PATH}/pylon-config --cflags-only-I
    OUTPUT_VARIABLE _pylon_INCLUDE)
string(REPLACE "-I" "" _pylon_INCLUDE ${_pylon_INCLUDE})
string(STRIP ${_pylon_INCLUDE} _pylon_INCLUDE)
string(REPLACE " " ";" pylon_INCLUDE ${_pylon_INCLUDE})
execute_process(COMMAND ${pylon_BIN_PATH}/pylon-config --libs-only-other
    OUTPUT_VARIABLE _pylon_LDFLAGS)
string(STRIP ${_pylon_LDFLAGS} pylon_LDFLAGS)
execute_process(COMMAND ${pylon_BIN_PATH}/pylon-config --libdir
    OUTPUT_VARIABLE _pylon_LIB_DIR)
string(STRIP ${_pylon_LIB_DIR} pylon_LIB_DIR)
execute_process(COMMAND ${pylon_BIN_PATH}/pylon-config --libs-only-l
    OUTPUT_VARIABLE _pylon_LIBS)
string(REPLACE "-l" "" _pylon_LIBS ${_pylon_LIBS})
string(STRIP ${_pylon_LIBS} _pylon_LIBS)
string(REPLACE " " ";" pylon_LIBS ${_pylon_LIBS})

generate_dynamic_reconfigure_options(
    cfg/Camera.cfg
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES basler_camera
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Additional Linker flags for pylon
SET( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${pylon_LDFLAGS}")

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${pylon_INCLUDE})
link_directories( ${pylon_LIB_DIR} )

## Declare a C++ library
#add_library(basler_camera src/basler_camera_node.cpp)
#add_dependencies(basler_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(basler_camera_node src/basler_camera_node.cpp)
add_dependencies(basler_camera_node ${PROJECT_NAME}_gencfg)
add_dependencies(basler_camera_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(basler_camera_node
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  ${pylon_LIBS}
) 

