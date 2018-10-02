/** @file block_proc.h
*    @brief Define the ROS Block action server
*    
*    
*    @author Alexis CARE
*/
#ifndef BLOCK_PROC_H
#define BLOCK_PROC_H

#include "ros/ros.h"

#include <robot_watcher/Services/RobotServices.h>
#include "robot_watcher/RStatus/State.h"

#include "procedures_msgs/BlockAction.h"
#include "procedures_msgs/MoveAction.h"
#include "procedures_msgs/MPoint.h"
#include "procedures_msgs/PliersAction.h"
#include "procedures_msgs/APliers.h"

#include "can_msgs/Point.h"
#include "can_msgs/Finish.h"

#include "ai_msgs/SetSide.h"
#include "ai_msgs/RobotStatus.h"

#include "move/MoveType.h"
#include "block/GroupBlocks.h"
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include "action/action_define.h"

/**
 * @defgroup bloc The block package
 * @{
 */

typedef actionlib::SimpleActionServer<procedures_msgs::BlockAction>   BlockSrv;
typedef actionlib::SimpleActionClient<procedures_msgs::MoveAction>    ClientMove;
typedef actionlib::SimpleActionClient<procedures_msgs::PliersAction>  ClientPliers;

typedef boost::shared_ptr< ::procedures_msgs::MoveResult const>   MoveResultConstPtr;
typedef boost::shared_ptr< ::procedures_msgs::PliersResult const> PliersResultConstPtr;


/**
 * @brief      Class for block action server
 */
class Block
{
public:
  Block(std::string name);

private:
  bool side;
  uint8_t phase;
  int point;

  ros::NodeHandle nh;

  ros::Subscriber side_sub;

  BlockSrv act;
  ClientMove acM;
  ClientPliers acP;

  GroupBlocks objectif;

  void goalCB();
  void preemptCB();
  void sendMsg();

  template <class doneMsg>
  void DoneAction( const actionlib::SimpleClientGoalState& state, const doneMsg & result);
  void setSide(const ai_msgs::SetSide::ConstPtr& msg);
  void GetRobotStatus(const ai_msgs::RobotStatus::ConstPtr& msg);
};

/**
 * @}
 */

#endif
