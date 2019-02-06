#ifndef ATOMIC_ACTION_H
#define ATOMIC_ACTION_H

#include "action_manager/ActionPoint.h"
#include "action_manager/Action.hpp"

#include "ai_msgs/ComputeActionPoint.h"

#include <ros/ros.h>
#include "ai_msgs/Argument.h"

#include <string>
#include <vector>

/**
 * @brief class for atomic actions
 */
class AtomicAction : public Action {
public:
  AtomicAction(std::string name, std::string performer);

  std::string performer() const;

  std::vector<ai_msgs::Argument> getArgs() const;
  ActionPoint& actionPoint(Point& previousPoint) override;

  // Setters
  void addArg(ai_msgs::Argument arg);

  // Equality
  virtual bool equals(const Action& action) const override;
  virtual void display(std::ostream& os) const override;
  
private:
  std::string _performer;
  std::vector<ai_msgs::Argument> _args;
};


#endif
