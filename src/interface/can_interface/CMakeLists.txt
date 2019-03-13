cmake_minimum_required(VERSION 2.8.3)
project(can_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  # ROS c++ dependencies
  roscpp

  # Required messages
  std_msgs
  interface_msgs
  can_msgs

  # Other packages
  node_watcher
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs can_msgs node_watcher interface_msgs
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
add_executable(${PROJECT_NAME}_node src/generated/robot_interface_gen.cpp)

# Rename C++ executable without prefix
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

# Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

# [IMPORTANT]
# Code generation for src/robot_interface_node.cpp
add_custom_target(
  ${PROJECT_NAME}_generation
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/src/generate_interface.py ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/src/generated/robot_interface_gen.cpp
)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generation)