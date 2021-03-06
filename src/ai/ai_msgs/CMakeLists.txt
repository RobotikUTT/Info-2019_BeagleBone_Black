cmake_minimum_required(VERSION 2.8.3)
project(ai_msgs)

find_package(catkin REQUIRED COMPONENTS
  # Actions build requirements
  actionlib_msgs

  # Messages and services base dependencies
  message_generation
  std_msgs
  geometry_msgs
  args_lib
)

# Message containing only constants shared by multiple messages
add_message_files(DIRECTORY const
  FILES
  Side.msg
  NodeStatus.msg
  ActionStatus.msg
  Topics.msg
)

# Regular messages files
add_message_files(DIRECTORY msg
  FILES
  # Type definition
  ActionPoint.msg
  Shape.msg
  
  # Meaningfull message
  PointsScored.msg # replace by std_msgs/Int16
  RobotStatus.msg
  StartRobot.msg
  ProximityStop.msg
  NodeStatusUpdate.msg
  NodeRequirement.msg
  AwaitNodesResult.msg
)

# Services files
add_service_files(
  FILES
  NodeReadiness.srv
  DeclareZone.srv
  ComputeActionPoint.srv
  SetSchedulerState.srv
  GetMapSize.srv
  AwaitNodesRequest.srv
  GetSidedPoint.srv
)

# Actions files
add_action_files(
  FILES
  Perform.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs actionlib_msgs geometry_msgs args_lib)

# Declare dependencies for external packages
catkin_package(CATKIN_DEPENDS message_runtime args_lib std_msgs actionlib_msgs geometry_msgs)
