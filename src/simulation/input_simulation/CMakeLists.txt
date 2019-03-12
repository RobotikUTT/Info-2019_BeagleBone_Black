cmake_minimum_required(VERSION 2.8.3)
project(input_simulation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  # ROS defaults
  roscpp

  # Packages
  node_watcher

  # Messages
  ai_msgs
  can_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES input_simulation
  CATKIN_DEPENDS ai_msgs can_msgs node_watcher roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/input_simulation_node.cpp)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)