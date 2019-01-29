#ifndef COMPOSED_ACTION_H
#define COMPOSED_ACTION_H

#include "scheduler/Action.hpp"

#include <list>
#include <string>

class ComposedAction : public Action {
public:
  ComposedAction(std::string name, std::list<Action> actions);


  int points();
  Point startPoint();
private:
  Action descriptor;
  std::list<Action> _actions;
};

#endif
