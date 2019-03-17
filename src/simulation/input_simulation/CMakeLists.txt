cmake_minimum_required(VERSION 2.8.3)
project(input_simulation)

find_package(catkin REQUIRED COMPONENTS
  # ROS defaults
  rospy

  # Packages
  node_watcher

  # Messages
  ai_msgs
  interface_msgs
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES input_simulation
  CATKIN_DEPENDS ai_msgs interface_msgs node_watcher rospy
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
