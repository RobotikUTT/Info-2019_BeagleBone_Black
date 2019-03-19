
cmake_minimum_required(VERSION 2.8.3)
project(vision)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  # ROS c++ dependencies
  roscpp

  # Required messages
  ai_msgs
  interface_msgs

  # Other packages
  node_watcher
  ai_description
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp interface_msgs ai_msgs ai_description node_watcher
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
add_executable(${PROJECT_NAME}_node src/vision_node.cpp)

# Rename C++ executable without prefix
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

# Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link the executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

