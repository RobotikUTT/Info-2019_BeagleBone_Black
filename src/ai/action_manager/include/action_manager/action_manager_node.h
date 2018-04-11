#ifndef ACTION_MANAGER_NODE_H
#define ACTION_MANAGER_NODE_H

#include "action_manager/ActionClass.h"
#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include <list>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include "robot_watcher/Services/RobotServices.h"
#include "robot_watcher/RStatus/Side.h"
#include "robot_watcher/SetSide.h"

#define ACTIONS_FILE "action_manager/actions.config"

class ActionManager
{
public:
  std::list<ActionClass> actions;
  ActionManager(ros::NodeHandle *n); // init with actions.config file
  // ~ActionManager();

private:

  ros::Subscriber side_sub;

  ros::NodeHandle nh;

  bool side;

  std::list<ActionClass> action;

  void setSide (const robot_watcher::SetSide::ConstPtr&);
  void updatePriority();

  //PointAction getActionToDo(); //vias service

  void actionsInit(); //parse actions.config

};

#endif
