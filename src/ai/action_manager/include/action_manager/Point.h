#ifndef POINT_H
#define POINT_H
#include <iostream>

class Point{
public:
  int x;
  int y;
  Point();
  Point(int, int);
};

std::ostream& operator<<(std::ostream&, const Point&);

#endif
