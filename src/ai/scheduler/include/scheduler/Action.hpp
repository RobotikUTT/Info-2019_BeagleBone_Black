#ifndef ACTION_H
#define ACTION_H

#include "scheduler/Point.h"

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
 

  int priority(Point& robot_pos);
  int points();
  int state();
  std::string name();

  Point startPoint();

  bool operator< ( const Action &B) const;

protected:
  Action(std::string name);

  int _state;
  std::string _name;
  
};

std::ostream& operator<<(std::ostream& os, const Action& AC);



#endif
