/**  @file ActionPoint.h
*    @brief Class for ActionPoint
*    
*    
*    @author Alexis CARE
*/
#ifndef ACTION_POINT_H
#define ACTION_POINT_H

#include "action_manager/Point.h"

/**
 * @brief decription of where the action begins and ends
 */
class ActionPoint{
public:
  Point startPoint;
  Point endPoint;

  ActionPoint();
  ActionPoint(Point start, Point end);
  ActionPoint(int start_x, int start_y, int start_angle, int end_x, int end_y, int end_angle);

  int distance();
};

std::ostream& operator<<(std::ostream&, const ActionPoint&);

#endif
