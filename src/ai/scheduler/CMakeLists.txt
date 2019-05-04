cmake_minimum_required(VERSION 2.8.3)
project(scheduler)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  # ROS c++ dependencies
  rospy

  # Required messages
  ai_msgs
  
  # Other packages
  action_manager
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rospy ai_msgs action_manager
#  DEPENDS system_lib
)

# Testing
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  
  foreach(T
      # Put launch test files here
      )
     add_rostest(${T})
  endforeach()
endif()