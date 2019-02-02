#include "action_manager/AtomicAction.hpp"

AtomicAction::AtomicAction(std::string name, std::string performer) :
  Action(name), _performer(performer), _actionPoint(NULL) { }


// Getters
int AtomicAction::points() const {
  return _points;
}

std::string AtomicAction::performer() const {
  return _performer;
}

/**
 *  Return the action point according either to the previous action's actionPoint
 *  or to current coordinates
 */
ActionPoint* AtomicAction::actionPoint(Point& previousActionPoint) {
  if (_actionPoint == NULL) {
    ActionPoint point;

    // TODO compute action point with perfomer help

    if (/*performer.isActionPointStatic()*/true) {
      _actionPoint = &point;
      return _actionPoint;
    }
  } else {
    return _actionPoint;
  }
}


std::list<ai_msgs::Argument> AtomicAction::getArgs() const {
  return _args;
}

// Setters
void AtomicAction::addArg(ai_msgs::Argument arg) {
  _args.push_back(arg);
}

bool operator==(const AtomicAction& lhs, const AtomicAction& rhs) {
  // First try basic tests
  bool baseTests = lhs.points() == rhs.points() && // points
    lhs.isSync() == rhs.isSync() && // sync
    lhs.performer() == rhs.performer() && // perfomer
    lhs.name() == rhs.name(); // name

  if (!baseTests) return false;

  // Then test for arguments
  std::list<ai_msgs::Argument> largs = lhs.getArgs();
  std::list<ai_msgs::Argument> rargs = rhs.getArgs();

  // as many args
  if (largs.size() != rargs.size()) return false;

  for(const auto& lnext : largs) {
    for(const auto& rnext : rargs) {
      if (lnext.name == rnext.name && lnext.value != rnext.value) {
        return false;
      }
    }
  }

  return true;
}