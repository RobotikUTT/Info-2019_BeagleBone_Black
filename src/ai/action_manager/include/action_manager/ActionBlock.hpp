#ifndef COMPOSED_ACTION_H
#define COMPOSED_ACTION_H

#include "action_manager/Action.hpp"

#include <list>
#include <string>

class ActionBlock : public Action {
public:
  ActionBlock(Action descriptor, std::list<Action> actions);

  int points();
  ActionPoint* actionPoint(Point& previousActionPoint);

  int distanceToTravel(Point& robot_pos);
private:
  std::list<Action> _actions;
};

#endif
