cmake_minimum_required(VERSION 2.8.3)
project(ai_msgs)

find_package(catkin REQUIRED COMPONENTS
  # Actions build requirements
  actionlib_msgs

  # Messages and services base dependencies
  message_generation
  std_msgs

  # Additional requirements
  procedures_msgs
)

# Message containing only constants shared by multiple messages
add_message_files(DIRECTORY const
  FILES
  Side.msg
  NodeStatus.msg
  ActionStatus.msg
)

# Regular messages files
add_message_files(DIRECTORY msg
  FILES
  PointsScored.msg
  RobotStatus.msg
  StartRobot.msg
  ProximityStop.msg
  Point2D.msg
  Argument.msg
  NodeStatusUpdate.msg
  NodeRequirement.msg
  AwaitNodesResult.msg
)

# Services files
add_service_files(
  FILES
  NodeReadiness.srv
  ObjectManager.srv
  ComputeActionPoint.srv
  SetSchedulerState.srv
  AwaitNodesRequest.srv
)

# Actions files
add_action_files(
  FILES
  Perform.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs actionlib_msgs procedures_msgs)

# Declare dependencies for external packages
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
