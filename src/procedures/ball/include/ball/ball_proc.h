#ifndef BALL_PROC_H
#define BALL_PROC_H

#include "ros/ros.h"
#include "ball/GroupBalls.h"
#include "action/action_define.h"
#include "procedures_msgs/MoveAction.h"
#include "move/MoveType.h"
#include "procedures_msgs/BallAction.h"
#include "procedures_msgs/MPoint.h"
#include "procedures_msgs/CanonAction.h"
#include "ai_msgs/SetSide.h"
#include "ai_msgs/RobotStatus.h"
#include <robot_watcher/Services/RobotServices.h>
#include "robot_watcher/RStatus/State.h"
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionServer<procedures_msgs::BallAction>  BallSrv;
typedef actionlib::SimpleActionClient<procedures_msgs::MoveAction> ClientMove;
typedef actionlib::SimpleActionClient<procedures_msgs::CanonAction> ClientCanon;

typedef boost::shared_ptr< ::procedures_msgs::MoveResult const> MoveResultConstPtr;
typedef boost::shared_ptr< ::procedures_msgs::CanonResult const> CanonResultConstPtr;


class  Ball
{
public:
   Ball(std::string name);

private:
  bool side;
  uint8_t phase;
  int point;

  ros::NodeHandle nh;

  // ros::Subscriber finish_sub;
  ros::Subscriber side_sub;

  // ros::Publisher ARDUINO_pliers_pub;
  // ros::Publisher ;
  // ros::Publisher ;
  // ros::Publisher ;

  BallSrv act;
  ClientMove acM;
  ClientCanon acC;

  // std::vector<Group Balls> fifo;
  GroupBalls objectif;

  void goalCB();
  void preemptCB();
  // void analysisCB(const can_msgs::Finish::ConstPtr&);
  void sendMsg();

  template <class doneMsg>
  void DoneAction( const actionlib::SimpleClientGoalState& state, const doneMsg & result);
  void setSide(const ai_msgs::SetSide::ConstPtr& msg);
  void GetRobotStatus(const ai_msgs::RobotStatus::ConstPtr& msg);


  // procedures_msgs::XXFeedback feedback
  // procedures_msgs::XXResult result
};


#endif
