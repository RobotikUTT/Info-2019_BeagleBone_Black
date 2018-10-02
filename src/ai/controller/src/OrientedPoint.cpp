#include "controller/OrientedPoint.h"

/**
 * @brief      Constructs the object.
 *
 * @param[in]  _x      X pos
 * @param[in]  _y      Y pos
 * @param[in]  _angle  The angle
 */
OrientedPoint::OrientedPoint(int _x, int _y, int _angle){
  x = _x;
  y = _y;
  angle = _angle;
}

/**
 * @brief      Constructs the object.
 *
 *angle = 0
 *
 * @param[in]  _x    X pos
 * @param[in]  _y    Y pos
 */
OrientedPoint::OrientedPoint(int _x, int _y){
  x = _x;
  y = _y;
  angle = 0;
}

/**
 * @brief      Constructs the default object.
 * 
 * x = y = angle = 0
 */
OrientedPoint::OrientedPoint(){
  x = 0;
  y = 0;
  angle = 0;
}

/**
 * @brief      OrientedPoint to stream
 *
 * @param      os    The operating system
 * @param[in]  P     OrientedPoint object
 *
 * @return     stream
 */
std::ostream& operator<<(std::ostream& os, const OrientedPoint& P)
{
    os << "point : { x: " << P.x << " ; y: " << P.y << " ; y: " << P.angle << " }";
    return os;
}
