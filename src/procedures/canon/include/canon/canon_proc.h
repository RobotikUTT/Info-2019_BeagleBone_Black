/** @file canon_proc.h
*    @brief Define the ROS canon action server
*    
*    
*    @author Alexis CARE
*/
#ifndef CANON_PROC_H
#define CANON_PROC_H

#include "ros/ros.h"

#include <robot_watcher/Services/RobotServices.h>
#include "robot_watcher/RStatus/State.h"

#include "can_msgs/ThrowBalls.h"
#include "can_msgs/Finish.h"

#include "procedures_msgs/CanonAction.h"

#include "ai_msgs/RobotStatus.h"

#include <actionlib/server/simple_action_server.h>
#include "action/action_define.h"

/**
 * @defgroup Canon The canon action package
 * @{
 */

typedef actionlib::SimpleActionServer<procedures_msgs::CanonAction> CanonSrv;


/**
 * @brief      Class for canon action server.
 */
class Canon
{
public:
  Canon(std::string name);

private:

  ros::NodeHandle nh;

  ros::Subscriber finish_sub;
  ros::Subscriber robot_watcher_sub;

  ros::Publisher ARDUINO_canon_pub;
  CanonSrv act;

  void goalCB();
  void preemptCB();
  void analysisCB(const can_msgs::Finish::ConstPtr&);
  inline void sendMsg();
  void GetRobotStatus(const ai_msgs::RobotStatus::ConstPtr& msg);
};

/**
 * @}
 */
#endif
