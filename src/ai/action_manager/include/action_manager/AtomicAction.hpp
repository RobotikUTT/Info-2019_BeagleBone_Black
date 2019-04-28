#ifndef ATOMIC_ACTION_H
#define ATOMIC_ACTION_H

#include "action_manager/Action.hpp"
#include "action_manager/Argumentable.hpp"

#include "ai_msgs/ActionPoint.h"
#include "ai_msgs/ComputeActionPoint.h"

#include <ros/ros.h>
#include "ai_msgs/Argument.h"

#include <string>
#include <vector>


/**
 * @brief class for atomic actions
 */
class AtomicAction : public Action, public Argumentable {
public:
  AtomicAction(std::string name, std::string performer);

  std::string performer() const;

  ActionPoint& actionPoint(Pose2D& previousPoint) override;

  // Equality
  //virtual bool equals(const Action& action) const override;
  virtual void display(std::ostream& os) const override;
  
private:
  std::string _performer;
};

typedef std::shared_ptr<AtomicAction> AtomicActionPtr;

#endif
