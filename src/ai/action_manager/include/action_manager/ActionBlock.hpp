#ifndef COMPOSED_ACTION_H
#define COMPOSED_ACTION_H

#include "action_manager/Action.hpp"
#include "action_manager/AtomicAction.hpp"

#include <list>
#include <string>

class ActionBlock : public Action {
public:
  ActionBlock(Action descriptor, std::list<Action> actions);

  int points() const;
  std::list<Action> subactions() const;
  
  ActionPoint* actionPoint(Point& previousActionPoint);

  int distanceToTravel(Point& robot_pos);
private:
  std::list<Action> _actions;
};

bool operator==(const ActionBlock& lhs, const ActionBlock& rhs);

#endif
