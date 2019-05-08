cmake_minimum_required(VERSION 2.8.3)
project(idle)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  # ROS c++ dependencies
  roscpp

  # Message dependencies
  interface_msgs

  # Other packages
  action_manager
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES idle
  CATKIN_DEPENDS roscpp robot_interface action_manager interface_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ executable
add_executable(${PROJECT_NAME}_proc src/idle_proc.cpp)

# Rename C++ executable without prefix
set_target_properties(${PROJECT_NAME}_proc PROPERTIES OUTPUT_NAME proc PREFIX "")

# Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME}_proc ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_proc
  ${catkin_LIBRARIES}
)