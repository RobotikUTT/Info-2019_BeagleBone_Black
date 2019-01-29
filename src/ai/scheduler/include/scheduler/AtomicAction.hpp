#ifndef ATOMIC_ACTION_H
#define ATOMIC_ACTION_H

#include "scheduler/ActionPoint.h"
#include "scheduler/Action.hpp"

#include <ros/ros.h>
#include "ai_msgs/Argument.h"

#include <string>
#include <list>

/**
 * @brief class for atomic actions
 */
class AtomicAction : public Action {
public:
  AtomicAction(std::string name, std::string performer);

  std::string performer();

  int points();
  Point startPoint();

  std::list<Argument> getArgs();

  // Setters
  void addArg(Argument arg);

private:
  std::string _performer;
  std::list<Argument> _args;
};

// TODO (or use a library?)
//std::ostream& operator<<(std::ostream& os, const AtomicAction& obj);

#endif
