cmake_minimum_required(VERSION 2.8.3)
project(basler_camera)

find_package(catkin REQUIRED COMPONENTS roscpp image_transport cv_bridge)
find_package(OpenCV REQUIRED)

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
SET( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -Wl,-E")

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} $ENV{PYLON_ROOT}/include $ENV{GENICAM_ROOT_V2_3}/library/CPP/include)
link_directories(
  $ENV{PYLON_ROOT}/lib64 
  $ENV{GENICAM_ROOT_V2_3}/bin/Linux64_x64
  $ENV{GENICAM_ROOT_V2_3}/bin/Linux64_x64/GenApi/Generic
  $ENV{GENICAM_ROOT_V2_3}/library/CPP/lib/Linux64_x64
)

## Declare a C++ library
#add_library(basler_camera src/basler_camera_node.cpp)
#add_dependencies(basler_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(basler_camera_node src/basler_camera_node.cpp)
add_dependencies(basler_camera_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(basler_camera_node
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  pylonbase
  pylonutility
  GenApi_gcc40_v2_3
  GCBase_gcc40_v2_3
) 

#############
## Install ##
############# 

#install(TARGETS basler_camera basler_camera_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
# )

#install(DIRECTORY launch/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION/launch}
#)

