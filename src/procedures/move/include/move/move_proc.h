#ifndef MOVE_PROC_H
#define MOVE_PROC_H

#include "ros/ros.h"
#include <vector>

#include <robot_watcher/Services/RobotServices.h>
#include "robot_watcher/RStatus/State.h"

#include "can_msgs/Point.h"
#include "can_msgs/Finish.h"
#include "can_msgs/Status.h"

#include "procedures_msgs/MoveAction.h"

#include "ai_msgs/RobotStatus.h"

#include "action/action_define.h"
#include "move/MoveType.h"
#include "move/MovePoint.h"
#include <actionlib/server/simple_action_server.h>

#include "robot_interface/protocol.h"


typedef actionlib::SimpleActionServer<procedures_msgs::MoveAction> MoveSrv;


class Move
{
public:
  Move(std::string name);

private:

  ros::NodeHandle nh;

  ros::Subscriber finish_sub;
  ros::Subscriber robot_watcher_sub;

  ros::Publisher STMGoToAngle_pub;
  ros::Publisher STMGoTo_pub;
  ros::Publisher STMRotation_pub;
  ros::Publisher STMRotationNoModulo_pub;
  ros::Publisher STM_AsserManagement_pub;
  ros::Timer TimerTimeout;

  MoveSrv act;

  std::vector<MovePoint> fifo;

  void goalCB();
  void preemptCB();
  void analysisCB(const can_msgs::Finish::ConstPtr&);
  inline void sendMsg();
  void GetRobotStatus(const ai_msgs::RobotStatus::ConstPtr& msg);
  void TimeoutCallback(const ros::TimerEvent&);

};
#endif
