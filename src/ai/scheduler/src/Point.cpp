#include "scheduler/Point.h"

Point::Point(){
  this->x = 0;
  this->y = 0;
}

Point::Point(int _x, int _y){
  this->x = _x;
  this->y = _y;
}

int Point::manhattanDist(Point& P){
  Point temp = *this - P;
  return temp.manhattanDist();
}

int Point::manhattanDist(){
  return std::abs(this->x + this->y);
}

Point Point::operator- (const Point& P){
  return Point (this->x - P.x, this->y - P.y);
}

std::ostream& operator<<(std::ostream& os, const Point& P)
{
    os << "point : { x: " << P.x << " ; y: " << P.y << " }";
    return os;
}
