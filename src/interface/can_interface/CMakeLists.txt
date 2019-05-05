cmake_minimum_required(VERSION 2.8.3)
project(can_interface)

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
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rospy std_msgs node_watcher interface_msgs args_lib
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS src/can_interface_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest)
  foreach(T
      test/test-parsing.launch)
     add_rostest(${T})
  endforeach()
endif()