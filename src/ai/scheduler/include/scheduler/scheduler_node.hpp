
/**  @file scheduler_node.h
*    @brief Define the ROS node Scheduler
*    
*    
*    @author Alexis CARE
*/

#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <ros/ros.h>

#include "robot_watcher/Services/RobotServices.h"
#include "robot_watcher/RStatus/Side.h"

#include "ai_msgs/SetSide.h"
#include "ai_msgs/GetActionToDo.h"
#include "ai_msgs/CurrentActionDone.h"

#include "scheduler/ActionsParser.hpp"
#include "scheduler/ActionFilePath.hpp"

/**
 * @brief      Class for scheduler wich scedule actions To Do.
 */
class Scheduler{
public:
  Scheduler(ros::NodeHandle* n);

private:
  ros::NodeHandle nh;

  ros::Subscriber side_sub;

  ros::ServiceServer action_srv;
  ros::ServiceServer actionD_srv;

  bool side;

  void setSide(const ai_msgs::SetSide::ConstPtr& msg);

  bool getActionToDo(ai_msgs::GetActionToDo::Request &req,
                      ai_msgs::GetActionToDo::Response &res); //vias service

  bool currentActionDone(ai_msgs::CurrentActionDone::Request &req,
                      ai_msgs::CurrentActionDone::Response &res); //vias service

};
#endif
