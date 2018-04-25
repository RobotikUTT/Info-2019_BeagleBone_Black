#ifndef MOVE_PROC_H
#define MOVE_PROC_H

#include "ros/ros.h"
// #include "ros_can/can"
#include "procedures_msgs/MoveAction.h"
#include "can_msgs/Point.h"
#include "can_msgs/Finish.h"
#include <vector>
#include "move/MoveType.h"
#include <robot_watcher/Services/RobotServices.h>
#include <actionlib/server/simple_action_server.h>

#include "move/MovePoint.h"

typedef actionlib::SimpleActionServer<procedures_msgs::MoveAction> MoveSrv;


class Move
{
public:
  Move(std::string name);

private:

  ros::NodeHandle nh;

  ros::Subscriber sub;

  ros::Publisher STMGoToAngle_pub; //"/STM/GoToAngle"
  ros::Publisher STMGoTo_pub; //"/STM/GoTo"
  ros::Publisher STMRotation_pub; //"/STM/Rotation"
  ros::Publisher STMRotationNoModulo_pub; //"/STM/RotationNoModulo";

  MoveSrv act;

  std::vector<MovePoint> fifo;

  void goalCB();
  void preemptCB();
  void analysisCB(const can_msgs::Finish::ConstPtr&);
  inline void sendMsg();


  // procedures_msgs::XXFeedback feedback
  // procedures_msgs::XXResult result
};


#endif
