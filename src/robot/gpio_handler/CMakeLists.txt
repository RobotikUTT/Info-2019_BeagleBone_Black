cmake_minimum_required(VERSION 2.8.3)
project(gpio_handler)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  # ROS c++ and python dependencies
  rospy
  
  # Message packages
  std_msgs
  ai_msgs
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs ai_msgs
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)
