#ifndef COMPOSED_ACTION_H
#define COMPOSED_ACTION_H

#include "action_manager/Action.hpp"
#include "action_manager/AtomicAction.hpp"

#include <list>
#include <string>
#include <memory>

typedef std::shared_ptr<Action> ActionPtr;

class ActionBlock : public Action {
public:
  ActionBlock(std::string name);

  int points() const override;
  std::list<ActionPtr> subactions() const;
  void addAction(ActionPtr action);
  
  ActionPoint* actionPoint(Point& previousActionPoint) override;

  double distanceToTravel(Point& robot_pos) override;

  // Equality
  virtual bool equals(const Action& action) const override;
  virtual void display(std::ostream& os) const override;
private:
  std::list<ActionPtr> _actions;
};

#endif
