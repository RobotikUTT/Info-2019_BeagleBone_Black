#ifndef POINT_H
#define POINT_H
#include <iostream>
#include <cmath>

class Point{
public:
  int x;
  int y;
  int angle;
  Point();
  Point(int _x, int _y, int _angle);
  Point(int _x, int _y);
  int manhattanDist(Point& P);
  int manhattanDist();

  Point operator- (const Point& P);
};

std::ostream& operator<<(std::ostream&, const Point&);

#endif
