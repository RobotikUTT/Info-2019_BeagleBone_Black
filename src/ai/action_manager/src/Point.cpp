/** @file Point.cpp
*    @brief class for Point.
*    
*    
*    @author Alexis CARE
*/
#include "action_manager/Point.hpp"

/**
 * @brief      Constructs the object.
 */
Point::Point(){
  this->x = 0;
  this->y = 0;
  this->angle = 0;
}

/**
 * @brief      Constructs the object.
 *
 * @param[in]  _x      X pos
 * @param[in]  _y      Y pos
 * @param[in]  _angle  angle [-Pi, Pi]
 */
Point::Point(int _x, int _y, int _angle){
  this->x = _x;
  this->y = _y;
  this->angle = _angle;
}

/**
 * @brief      Constructs the object.
 *
 * @param[in]  _x    X pos
 * @param[in]  _y    Y pos
 */
Point::Point(int _x, int _y){
  this->x = _x;
  this->y = _y;
  this->angle = 0;
}

/**
 * @brief      Calculate the manathan distance between two Points
 *
 * @param      P     The other Point
 *
 * @return     The manathan distance
 */
int Point::manhattanDist(Point& P){
  Point temp = *this - P;
  return temp.manhattanDist();
}

/**
 * @brief      Calculate the manathan distance to the Point
 *
 * @return     The manathan distance
 */
int Point::manhattanDist(){
  return std::abs(this->x + this->y);
}

/**
 * @brief      Operator '-' between two Point
 *
 * @param[in]  P     The other Point
 *
 * @return     A Point vector
 */
Point Point::operator- (const Point& P){
  return Point (this->x - P.x, this->y - P.y);
}

/**
 * @brief      Transform a Point to a stream
 *
 * @param      os    The operating system
 * @param[in]  P     A point
 *
 * @return     A stream
 */
std::ostream& operator<<(std::ostream& os, const Point& P){
    os << "point : { x: " << P.x << " ; y: " << P.y << " }";
    return os;
}

bool operator ==(const Point &a, const Point &b) {
  return a.x == b.x && a.y == b.y && a.angle == b.angle;
}