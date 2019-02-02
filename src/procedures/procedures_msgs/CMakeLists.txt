cmake_minimum_required(VERSION 2.8.3)
project(procedures_msgs)

find_package(catkin REQUIRED COMPONENTS
  # Actions build requirements
  actionlib_msgs

  # Messages and services base dependencies
  message_generation
  std_msgs
)

# Files to be built
add_message_files(
  FILES
  MPoint.msg
  OrPoint.msg
)

# Actions
add_action_files(
  FILES
  Move.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

# Declare dependencies for external packages
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)