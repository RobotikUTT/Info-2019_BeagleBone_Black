#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <ros/ros.h>
#include "robot_watcher/Services/RobotServices.h"
#include "robot_watcher/RStatus/Side.h"
#include "robot_watcher/SetSide.h"
#include "robot_watcher/GetActionToDo.h"
#include "scheduler/ActionManager.h"


class Scheduler{
public:
  Scheduler(ros::NodeHandle* n);

private:
  ros::Subscriber side_sub;

  ros::ServiceServer action_srv;

  ros::NodeHandle nh;

  ActionManager actionManager;

  bool side;

  void setSide(const robot_watcher::SetSide::ConstPtr& msg);

  bool getActionToDo(robot_watcher::GetActionToDo::Request &req,
                      robot_watcher::GetActionToDo::Response &res); //vias service



};
#endif
