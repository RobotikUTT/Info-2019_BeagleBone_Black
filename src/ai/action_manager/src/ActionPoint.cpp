/** @file ActionPoint.cpp
*    @brief Class for ActionPoint.
*    
*    
*    @author Alexis CARE
*/
#include "action_manager/ActionPoint.h"


/**
 * @brief      Constructs the default object.
 */
ActionPoint::ActionPoint(){
  this->startPoint = Point(0,0,0);
  this->endPoint = Point(0,0,0);
}

/**
 * @brief      Constructs the object.
 *
 * @param[in]  start  The start point
 * @param[in]  end    The end point
 */
ActionPoint::ActionPoint(Point start, Point end){
  this->startPoint = start;
  this->endPoint = end;
}

/**
 * @brief      Constructs the object.
 *
 * @param[in]  start_x      The start x pos
 * @param[in]  start_y      The start y pos
 * @param[in]  start_angle  The start angle
 * @param[in]  end_x        The end x pos
 * @param[in]  end_y        The end y pos
 * @param[in]  end_angle    The end angle
 */
ActionPoint::ActionPoint(int start_x, int start_y, int start_angle, int end_x, int end_y, int end_angle){
  this->startPoint = Point(start_x, start_y, start_angle);
  this->endPoint = Point(end_x, end_y, end_angle);
}

int ActionPoint::distance() {
  return this->startPoint.manhattanDist(this->endPoint);
}

/**
 * @brief      Transform a ActionPoint to a stream
 *
 * @param      os    The operating system
 * @param[in]  AP    The ActionPoint
 *
 * @return     A stream
 */
std::ostream& operator<<(std::ostream& os, const ActionPoint& AP)
{
    os <<  "ActionPoint : { \n\t start_" << AP.startPoint << "\n\t end_" << AP.endPoint << "\n}";
    return os;
}
