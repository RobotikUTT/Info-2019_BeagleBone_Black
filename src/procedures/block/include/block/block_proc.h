#ifndef BLOCK_PROC_H
#define BLOCK_PROC_H

#include "ros/ros.h"
#include "block/GroupBlocks.h"
#include "procedures_msgs/BlockAction.h"
#include "procedures_msgs/MoveAction.h"
#include "procedures_msgs/MPoint.h"
#include "can_msgs/Point.h"
#include "can_msgs/Finish.h"
#include "can_msgs/ActionPliers.h"
#include "ai_msgs/SetSide.h"
#include "ai_msgs/RobotStatus.h"
#include <robot_watcher/Services/RobotServices.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include "action/action_define.h"
#include "move/MoveType.h"
#include "robot_watcher/RStatus/State.h"

typedef actionlib::SimpleActionServer<procedures_msgs::BlockAction> BlockSrv;
typedef actionlib::SimpleActionClient<procedures_msgs::MoveAction> ClientMove;

typedef boost::shared_ptr< ::procedures_msgs::MoveResult const> MoveResultConstPtr;


class Block
{
public:
  Block(std::string name);

private:
  bool side;
  uint8_t phase;
  int point;

  ros::NodeHandle nh;

  ros::Subscriber finish_sub;
  ros::Subscriber side_sub;

  ros::Publisher ARDUINO_pliers_pub;
  // ros::Publisher ;
  // ros::Publisher ;
  // ros::Publisher ;

  BlockSrv act;
  ClientMove acM;

  // std::vector<GroupBlocks> fifo;
  GroupBlocks objectif;

  void goalCB();
  void preemptCB();
  void analysisCB(const can_msgs::Finish::ConstPtr&);
  void sendMsg();

  void DoneMove( const actionlib::SimpleClientGoalState& state, const MoveResultConstPtr & result);
  void setSide(const ai_msgs::SetSide::ConstPtr& msg);
  void GetRobotStatus(const ai_msgs::RobotStatus::ConstPtr& msg);


  // procedures_msgs::XXFeedback feedback
  // procedures_msgs::XXResult result
};


#endif
