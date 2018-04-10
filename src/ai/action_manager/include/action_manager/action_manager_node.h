#ifndef ACTION_MANAGER_NODE_H
#define ACTION_MANAGER_NODE_H

#include "action_manager/ActionClass.h"
#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include <list>
#include <iostream>

#define ACTIONS_FILE "action_manager/actions.config"

class ActionManager
{
public:
  std::list<ActionClass> actions;
  ActionManager() // init with actions.config file
  ~ActionManager()


private:

  ros::Subscriber side_sub;

  std::vector<ActionPoint> v;

  void setSide (const robot_watcher::SetSide::ConstPtr&);
  void updatePriority();

  PointAction getActionToDo(); //vias service

  void actionsInit(); //parse actions.config

};

#endif
