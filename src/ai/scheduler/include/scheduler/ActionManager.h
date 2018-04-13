#ifndef ACTION_MANAGER_NODE_H
#define ACTION_MANAGER_NODE_H

#include "scheduler/ActionClass.h"
#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include <list>
#include <iostream>
#include <string>
#include <limits>

class ActionManager
{
public:
  std::list<ActionClass> actions;
  ActionManager(const char*); // init with actions.config file
  ActionManager();
  void changeSide ();

  std::string getActionToDo();
  void updatePriority(Point robot_pos);

private:
  std::list<ActionClass> action;

  void actionsInit(const char*); //parse actions.config

};

#endif
