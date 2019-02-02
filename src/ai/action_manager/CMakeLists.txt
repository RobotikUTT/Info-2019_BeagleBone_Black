cmake_minimum_required(VERSION 2.8.3)
project(action_manager)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  # ROS c++ dependencies
  roscpp

  # Messages packages
  procedures_msgs
  ai_msgs
  can_msgs

  # Other packages
  robot_watcher
  actionlib
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES action_manager
  CATKIN_DEPENDS roscpp procedures_msgs ai_msgs can_msgs robot_watcher actionlib
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
  src/Action.cpp
  src/AtomicAction.cpp
  src/ActionBlock.cpp
  src/ActionPoint.cpp
  src/Point.cpp
  src/ActionPerformer.cpp
)

# Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
