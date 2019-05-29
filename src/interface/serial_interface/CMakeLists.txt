cmake_minimum_required(VERSION 2.8.3)
project(serial_interface)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  # ROS python dependencies
  rospy

  # Required messages
  std_msgs
  interface_msgs
  
  # Other packages
  node_watcher
  interface_description
  args_lib
  can_interface
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rospy std_msgs node_watcher interface_msgs args_lib can_interface
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS src/serial_interface_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
