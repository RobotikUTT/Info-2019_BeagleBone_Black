#ifndef ACTION_POINT_H
#define ACTION_POINT_H

#include "scheduler/Point.h"

class ActionPoint{
public:
  Point startPoint;
  Point endPoint;
  ActionPoint();
  ActionPoint(Point, Point);
  ActionPoint(int, int, int, int);
};

std::ostream& operator<<(std::ostream&, const ActionPoint&);

#endif