#ifndef ACTION_MANAGER_NODE_H
#define ACTION_MANAGER_NODE_H

#include "scheduler/ActionClass.h"
#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include "action/action_define.h"
#include <list>
#include <iostream>
#include <string>
#include <limits>
#include "ai_msgs/GetActionToDo.h"

class ActionManager
{
public:
  // std::list<ActionClass> actions;
  ActionManager(const char*); // init with actions.config file
  ActionManager();
  void changeSide ();

  void getActionToDo(ai_msgs::GetActionToDo::Response &);
  void updatePriority(Point robot_pos);
  void currentActionDone(bool done);

private:
  std::list<ActionClass> action;
  std::list<ActionClass>::iterator current_action;

  void actionsInit(const char*); //parse actions.config

};

#endif
