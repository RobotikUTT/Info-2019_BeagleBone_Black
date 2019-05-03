cmake_minimum_required(VERSION 2.8.3)
project(action_manager)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  # ROS c++ dependencies
  roscpp
  rospy

  # Messages packages
  ai_msgs
  interface_msgs

  # Other packages
  actionlib
  node_watcher
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy ai_msgs interface_msgs actionlib node_watcher
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
  src/Argumentable.cpp
  src/ActionBlock.cpp
  src/ActionPerformer.cpp
  src/PerformClient.cpp
)

# Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Testing
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  
  foreach(T
      test/test-actions.launch
      test/test-parsing.launch)
     add_rostest(${T})
  endforeach()
endif()