cmake_minimum_required(VERSION 2.8.3)
project(interface_description)

find_package(catkin REQUIRED COMPONENTS
  # Messages and services base dependencies
  message_generation
  std_msgs
)

# Constants
add_message_files(DIRECTORY const
  FILES
  InterfaceTopics.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)

# Declare dependencies for external packages
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

include_directories()
