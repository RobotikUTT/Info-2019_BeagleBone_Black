
cmake_minimum_required(VERSION 2.8.3)
project(vision)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  # ROS c++ dependencies
  rospy

  # Required messages
  ai_msgs
  interface_msgs

  # Other packages
  args_lib
  node_watcher
  ai_description
  action_manager
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rospy action_manager interface_msgs ai_msgs ai_description node_watcher
#  DEPENDS system_lib
)

# Testing
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  # Tests to perform
  foreach(T
      test/test-parsing.launch)
     add_rostest(${T})
  endforeach()
endif()