#ifndef COMPOSED_ACTION_H
#define COMPOSED_ACTION_H

#include "action_manager/Action.hpp"
#include "action_manager/AtomicAction.hpp"

#include <list>
#include <string>
#include <memory>

class ActionBlock;

class ActionBlock : public Action, public std::enable_shared_from_this<ActionBlock> {
public:
  ActionBlock(std::string name);

  int points() const override;
  std::list<ActionPtr> subactions() const;
  void addAction(ActionPtr action);
  
  ActionPoint& actionPoint(Point& previousPoint) override;

  double distanceToTravel(Point& robot_pos) override;

  // Inherited setter for action state
  void setState(int state) override;

  // Equality
  virtual bool equals(const Action& action) const override;
  virtual void display(std::ostream& os) const override;
private:
  std::list<ActionPtr> _actions;
};

typedef std::shared_ptr<ActionBlock> ActionBlockPtr;

#endif
