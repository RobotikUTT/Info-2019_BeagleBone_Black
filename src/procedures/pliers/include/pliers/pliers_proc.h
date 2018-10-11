/** @file pliers_proc.h
*    @brief Define the ROS Pliers action server
*    
*    
*    @author Alexis CARE
*/
#ifndef PLIERS_PROC_H
#define PLIERS_PROC_H

#include "ros/ros.h"
#include <vector>

#include <robot_watcher/Services/RobotServices.h>
#include "robot_watcher/RStatus/State.h"

#include "procedures_msgs/PliersAction.h"

#include "can_msgs/ActionPliers.h"
#include "can_msgs/Finish.h"

#include "ai_msgs/RobotStatus.h"

#include "std_msgs/Int8.h"

#include "action/action_define.h"
#include "pliers/PliersCommand.h"
#include <actionlib/server/simple_action_server.h>

/**
 * @defgroup Pliers The pliers action package
 * @{
 */

typedef actionlib::SimpleActionServer<procedures_msgs::PliersAction> PliersSrv;


/**
 * @brief      Class for pliers action server
 */
class Pliers
{
public:
  Pliers(std::string name);

private:

  ros::NodeHandle nh;

  ros::Subscriber finish_sub;
  ros::Subscriber robot_watcher_sub;

  ros::Publisher ARDUINO_pliers_pub;
  ros::Publisher ARDUINO_Move_pliers_pub;
  PliersSrv act;

  std::vector<PliersCommand> fifo;

  void goalCB();
  void preemptCB();
  void analysisCB(const can_msgs::Finish::ConstPtr&);
  inline void sendMsg();
  void GetRobotStatus(const ai_msgs::RobotStatus::ConstPtr& msg);
};

/**
 * 
 * @}
 */
#endif
