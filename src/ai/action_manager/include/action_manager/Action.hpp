#ifndef ACTION_H
#define ACTION_H

#include "action_manager/ActionPoint.h"

#include "ai_msgs/ActionStatus.h"

#include <iostream>
#include <string>
#include <sstream>
#include <memory>

using ai_msgs::ActionStatus;

/**
 * class for a general action
 */
class Action {
public:
  // Setters
  void setBasePoints(int points);
  void setSync(bool sync);
  virtual void setState(int state);
  void setParent(std::shared_ptr<Action> parent);

  // Getters
  bool isSync() const;
  double priority(Point& robot_pos);
  int state() const;
  std::string name() const;
  int getBasePoints() const;

  virtual double distanceToTravel(Point& robot_pos);
  virtual int points() const;
  virtual ActionPoint& actionPoint(Point& previousPoint) = 0;
  
  // Equality and display
  virtual bool equals(const Action& b) const;
  virtual void display(std::ostream& os) const;
  
protected:
  Action(std::string name = "unnamed");

  std::shared_ptr<Action> _parent;

  int _state, _points;
  std::string _name;
  bool _sync;

  std::shared_ptr<ActionPoint> _actionPoint;
};

typedef std::shared_ptr<Action> ActionPtr;

std::string getActionPointService(std::string performer);
std::string getActionServer(std::string performer);
std::string getActionNodePath(std::string performer);

std::ostream& operator<<(std::ostream& os, const Action& ac);

#endif
