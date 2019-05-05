cmake_minimum_required(VERSION 2.8.3)
project(interface_msgs)

find_package(catkin REQUIRED COMPONENTS
  # Messages and services base dependencies
  message_generation
  std_msgs
  ai_msgs
)

# Generic messages (uses both in input and output)
add_message_files(DIRECTORY msg
  FILES
  CanData.msg
  Directions.msg
  StmMode.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs ai_msgs)

# Declare dependencies for external packages
catkin_package(CATKIN_DEPENDS message_runtime std_msgs ai_msgs)
