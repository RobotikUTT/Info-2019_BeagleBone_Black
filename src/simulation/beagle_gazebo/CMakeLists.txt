cmake_minimum_required(VERSION 2.8.3)
project(beagle_gazebo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  # ROS defaults
  roscpp

  # Packages
  node_watcher
  gazebo_ros
  hector_gazebo_plugins

  # Messages
  ai_msgs
  std_msgs
  interface_msgs
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES input_simulation
  CATKIN_DEPENDS ai_msgs std_msgs node_watcher roscpp gazebo_ros interface_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

# Gazebo plugins
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(beagle_gazebo_model SHARED src/robot_model_plugin.cpp)
target_link_libraries(beagle_gazebo_model ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})