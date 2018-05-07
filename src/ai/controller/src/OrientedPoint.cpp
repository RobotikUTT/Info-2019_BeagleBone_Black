#include "controller/OrientedPoint.h"

OrientedPoint::OrientedPoint(int _x, int _y, int _angle){
  x = _x;
  y = _y;
  angle = _angle;
}

OrientedPoint::OrientedPoint(int _x, int _y){
  x = _x;
  y = _y;
  angle = 0;
}

OrientedPoint::OrientedPoint(){
  x = 0;
  y = 0;
  angle = 0;
}

std::ostream& operator<<(std::ostream& os, const OrientedPoint& P)
{
    os << "point : { x: " << P.x << " ; y: " << P.y << " ; y: " << P.angle << " }";
    return os;
}
