#ifndef CONTROLLER_NODE_H
#define CONTROLLER_NODE_H

#include "ros/ros.h"
#include "robot_watcher/Services/RobotServices.h"

#include "robot_watcher/RStatus/State.h"

#include "ai_msgs/GetActionToDo.h"

//include action

typedef actionlib::SimpleActionClient<procedures_msgs::XX> ClientMove;

class Controller
{
public:
  Controller(ros::NodeHandle* n);


private:
  ros::ServiceClient client;

  ros::Subscriber status_sub;
  ros::Subscriber robot_pos_sub;

  ros::NodeHandle nh;

  //robot pos
  int16_t robot_pos_x;
  int16_t robot_pos_y;

  uint8_t robot_status;
  std::string action_name;

  ClientMove acM;
  // std::map<std::string, value> map;

  void GetRobotStatus(const ai_msgs::RobotStatus::ConstPtr& msg);
  void SetAction(const ai_msgs::RobotStatus::ConstPtr& msg);
};
#endif
