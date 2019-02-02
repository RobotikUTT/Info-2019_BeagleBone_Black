cmake_minimum_required(VERSION 2.8.3)
project(move)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  # ROS c++ dependencies
  roscpp

  # Required messages
  can_msgs
  ai_msgs
  procedures_msgs

  # Other packages
  actionlib
  robot_watcher
  robot_interface
  controller
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES move
  CATKIN_DEPENDS can_msgs ai_msgs procedures_msgs actionlib robot_watcher robot_interface controller
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/MovePoint.cpp
)

## Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(${PROJECT_NAME}_proc src/move_proc.cpp)

## Rename C++ executable without prefix
set_target_properties(${PROJECT_NAME}_proc PROPERTIES OUTPUT_NAME proc PREFIX "")

## Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME}_proc ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_proc
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_proc
  ${PROJECT_NAME}
)
