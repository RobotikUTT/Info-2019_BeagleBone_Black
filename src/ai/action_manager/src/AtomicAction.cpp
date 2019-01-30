#include "action_manager/AtomicAction.hpp"

AtomicAction::AtomicAction(std::string name, std::string performer) :
  Action(name), _performer(performer), _actionPoint(NULL) { }


// Getters
int AtomicAction::points() {
  return _points;
}

/**
 *  Return the action point according either to the previous action's actionPoint
 *  or to current coordinates
 */
ActionPoint* AtomicAction::actionPoint(ActionPoint* previousActionPoint) {
  if (_actionPoint == NULL) {
    ActionPoint point;

    // TODO compute action point with perfomer help

    if (/*performer.isActionPointStatic()*/true) {
      _actionPoint = &point
      return _actionPoint;
    }
  } else {
    return _actionPoint;
  }
}


std::list<ai_msgs::Argument> AtomicAction::getArgs() {
  return _args;
}

// Setters
void AtomicAction::addArg(ai_msgs::Argument arg) {
  _args.push_back(arg);
}