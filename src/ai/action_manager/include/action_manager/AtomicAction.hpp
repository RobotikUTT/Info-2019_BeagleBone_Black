#ifndef ATOMIC_ACTION_H
#define ATOMIC_ACTION_H

#include "action_manager/ActionPoint.h"
#include "action_manager/Action.hpp"

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

  std::string performer() const;

  std::list<ai_msgs::Argument> getArgs() const;
  ActionPoint* actionPoint(Point& previousActionPoint);

  // Setters
  void addArg(ai_msgs::Argument arg);

  // Equality
  virtual bool equals(const Action& action);
private:
  std::string _performer;
  std::list<ai_msgs::Argument> _args;

  ActionPoint* _actionPoint;
};

// TODO (or use a library?)
std::ostream& operator<<(std::ostream& os, const AtomicAction& obj);

#endif
