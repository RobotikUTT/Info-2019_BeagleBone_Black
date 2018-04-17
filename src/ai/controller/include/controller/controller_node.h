#ifndef CONTROLLER_NODE_H
#define CONTROLLER_NODE_H

#include "ros/ros.h"
#include "robot_watcher/Services/RobotServices.h"

#include "robot_watcher/RStatus/State.h"

#include "ai_msgs/GetActionToDo.h"

//include action


class Controller:
public:
  Controller(ros::NodeHandle* n);


private:
  ros::ServiceClient client;

  ros::Subscriber status_sub;

  ros::NodeHandle nh;

  uint8_t robot_status;

  GetRobotStatus(const ai_msgs::RobotStatus::ConstPtr& msg);

#endif
