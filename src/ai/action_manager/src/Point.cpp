#include "Point.h"

Point::Point(){
  this->x = 0;
  this->y = 0;
}

Point::Point(int _x, int _y){
  this->x = _x;
  this->y = _y;
}

std::ostream& operator<<(std::ostream& os, const Point& P)
{
    os << "point : { x: " << P.x << " ; y: " << P.y << " }";
    return os;
}
