cmake_minimum_required(VERSION 2.8.3)
project(pathfinding)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  ai_msgs
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES pathfiding
  CATKIN_DEPENDS rospy ai_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)