cmake_minimum_required(VERSION 2.8.3)
project(ai_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  actionlib_msgs
  std_msgs
  #procedures_msgs
)

add_message_files(DIRECTORY msg
  FILES
  PointsScored.msg
  NodesStatus.msg
  RobotStatus.msg
  SetSide.msg
  EmergencyStop.msg
  Point2D.msg
  Argument.msg
)

add_service_files(
  FILES
  GetActionToDo.srv
  NodeReadiness.srv
  CurrentActionDone.srv
  ObjectManager.srv
  ComputeActionPoint.srv
)

add_action_files(
  FILES
  Perform.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  procedures_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    #procedures_msgs
)

#add_custom_target (build_doc
#  rosdoc_lite ${CATKIN_DEVEL_PREFIX}/../src/ai/${PROJECT_NAME}
#)