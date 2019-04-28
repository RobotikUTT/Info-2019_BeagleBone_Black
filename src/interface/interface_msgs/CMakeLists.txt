cmake_minimum_required(VERSION 2.8.3)
project(interface_msgs)

find_package(catkin REQUIRED COMPONENTS
  # Messages and services base dependencies
  message_generation
  std_msgs
  geometry_msgs
)

# Generic messages (uses both in input and output)
add_message_files(DIRECTORY msg
  FILES
  DirectedPose.msg
  Speed.msg
  WheelsDistance.msg
  WheelsSpeed.msg
)

# Input messages (used when information come from outside ai)
add_message_files(DIRECTORY input
  FILES
  RobotBlocked.msg
  SonarDistance.msg
  ObjectOnMap.msg
  StmDone.msg
)

# Ouput messages (used when information come from ai to the outside)
add_message_files(DIRECTORY output
  FILES
  MoveOrder.msg
  Pid.msg
  Pwms.msg
  StmParams.msg
  StmMode.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

# Declare dependencies for external packages
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
