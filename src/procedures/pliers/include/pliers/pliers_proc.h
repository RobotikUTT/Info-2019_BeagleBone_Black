#ifndef PLIERS_PROC_H
#define PLIERS_PROC_H

#include "ros/ros.h"
// #include "ros_can/can"
#include "procedures_msgs/PliersAction.h"
#include "can_msgs/ActionPliers.h"
#include "can_msgs/Finish.h"
#include "ai_msgs/RobotStatus.h"
#include <vector>
#include "pliers/PliersCommand.h"
#include <robot_watcher/Services/RobotServices.h>
#include <actionlib/server/simple_action_server.h>
#include "action/action_define.h"
#include "robot_watcher/RStatus/State.h"

typedef actionlib::SimpleActionServer<procedures_msgs::PliersAction> PliersSrv;


class Pliers
{
public:
  Pliers(std::string name);

private:

  ros::NodeHandle nh;

  ros::Subscriber finish_sub;
  ros::Subscriber robot_watcher_sub;

  // ros::Publisher STMGoToAngle_pub; //"/STM/GoToAngle"
  // ros::Publisher STMGoTo_pub; //"/STM/GoTo"
  // ros::Publisher STMRotation_pub; //"/STM/Rotation"
  // ros::Publisher STMRotationNoModulo_pub; //"/STM/RotationNoModulo";
  ros::Publisher ARDUINO_pliers_pub;
  PliersSrv act;

  std::vector<PliersCommand> fifo;

  void goalCB();
  void preemptCB();
  void analysisCB(const can_msgs::Finish::ConstPtr&);
  inline void sendMsg();
  void GetRobotStatus(const ai_msgs::RobotStatus::ConstPtr& msg);


  // procedures_msgs::XXFeedback feedback
  // procedures_msgs::XXResult result
};


#endif
