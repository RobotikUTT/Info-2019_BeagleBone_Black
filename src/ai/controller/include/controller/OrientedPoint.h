#ifndef ORIENTED_POINT_H
#define ORIENTED_POINT_H
#include <iostream>


class OrientedPoint
{
public:
  int x;
  int y;
  int angle;

  OrientedPoint(int _x, int _y, int _angle);
  OrientedPoint(int _x, int _y);
  OrientedPoint();
  
};

std::ostream& operator<<(std::ostream&, const OrientedPoint&);
#endif
