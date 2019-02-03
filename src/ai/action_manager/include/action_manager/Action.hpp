#ifndef ACTION_H
#define ACTION_H

#include "action_manager/ActionPoint.h"

#include <string>

#define ACTION_IDLE 0
#define ACTION_RUNNING 1
#define ACTION_PAUSED 2
#define ACTION_IMPOSSIBLE 3
#define ACTION_DONE 4

/**
 * class for a general action
 */
class Action {
public:
  // Setters
  void setBasePoints(int points);
  void setSync(bool sync);

  // Getters
  bool isSync() const;
  int priority(Point& robot_pos);
  int state() const;
  std::string name() const;
  int getBasePoints() const;

  virtual int distanceToTravel(Point& robot_pos);
  virtual int points() const;
  virtual ActionPoint* actionPoint(Point& previousActionPoint);
  
  // Equality and display
  virtual bool equals(const Action& b) const;
  virtual void display(std::ostream& os) const;
  
protected:
  Action(std::string name = "unnamed action");

  int _state, _points;
  std::string _name;
  bool _sync;

  ActionPoint* _actionPoint;
};

std::ostream& operator<<(std::ostream& os, const Action& ac);

#endif
