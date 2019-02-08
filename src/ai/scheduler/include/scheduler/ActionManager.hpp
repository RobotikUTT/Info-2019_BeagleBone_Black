/**  @file ActionManager.h
*    @brief Class for ActionManager
*    
*    
*    @author Alexis CARE
*/
#ifndef ACTION_MANAGER_NODE_H
#define ACTION_MANAGER_NODE_H

#include <ros/ros.h>
#include "scheduler/action/Action.h"

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"

#include "ai_msgs/GetActionToDo.h"


#include <list>
#include <vector>
#include <iostream>
#include <string>
#include <limits>

/**
 * @brief      Class for action manager.
 */
class ActionManager
{
public:
  ActionManager(ActionPtr rootAction);
  void changeSide ();

  void currentActionDone(bool done);

private:
  std::list<ActionClass> action;
  std::list<ActionClass>::iterator current_action;

};

#endif
